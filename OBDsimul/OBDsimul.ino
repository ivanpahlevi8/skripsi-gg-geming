#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdio.h"
#include "stdlib.h"
#include "driver/twai.h"

void twai_setup_and_install(){
    //Initialize configuration structures using macro initializers
    twai_general_config_t g_config = {
        .mode = TWAI_MODE_NORMAL,
        .tx_io = GPIO_NUM_21,
        .rx_io = GPIO_NUM_22,
        .clkout_io = TWAI_IO_UNUSED,
        .bus_off_io = TWAI_IO_UNUSED,
        .tx_queue_len = 5,
        .rx_queue_len = 5,
        .alerts_enabled = TWAI_ALERT_NONE,
        .clkout_divider = 0
    };

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        printf("Driver installed\n");
    } else {
        printf("Failed to install driver\n");
        return;
    }

    // Start TWAI driver
    if (twai_start() == ESP_OK) {
        printf("Driver started\n");
    } else {
        printf("Failed to start driver\n");
        return;
    }
}

void new_message(twai_message_t *message, uint32_t id, uint8_t dlc, uint8_t *data)
{
    
    message->flags = TWAI_MSG_FLAG_NONE;
    message->identifier = id;
    message->data_length_code = dlc;
    for (int i = 0; i < dlc; i++) {
        message->data[i] = data[i];
    }
    printf("Message created\nID: %x DLC: %d Data:\t", message->identifier, message->data_length_code);
    for (int i = 0; i < message->data_length_code; i++) {
        printf("%d\t", message->data[i]);
    }
    printf("\n");
}

void transmit_message(twai_message_t *message)
{
    if (twai_transmit(message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        printf("Message queued for transmission\n");
    } else {
        printf("Failed to send message\n");
    }
}

void receive_message(twai_message_t *message)
{
    if (twai_receive(message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        printf("\n");
        printf("ID: %x DLC: %d Data: \t", message->identifier, message->data_length_code);
        for (int i = 0; i < message->data_length_code; i++) {
            //(message->extd)?printf("Extended ID"):printf("Standard ID");
            printf("%02x\t", message->data[i]);
        }
    } else {
        printf("Failed to receive message\n");
    }
}

void setup(){
  Serial.begin(115200);
}

void loop()
{
    
    twai_setup_and_install();
    twai_message_t message;
    twai_message_t message1;
    // Set the data to send
    uint8_t data[8] = {rand()+10 % 255, rand()+5 % 255, rand()+4 % 255, 
    rand() % 255, rand() % 255, rand() % 255, rand() % 255, rand() % 255};

    while(true){ 
    // Create a new message
    new_message(&message, 352252193, 8, data);

    // Transmit the message to a queue
    transmit_message(&message);

    // Receive the message from the queue
    //receive_message(&message1);

    // Wait for 1 second
    vTaskDelay(100 / portTICK_PERIOD_MS);
    }

}