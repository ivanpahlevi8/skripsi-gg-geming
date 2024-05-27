#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdio.h"
#include "stdlib.h"
#include "driver/twai.h"
#include "SD.h"
#include "SPI.h"
#include "FS.h"

#define button1 14
#define button2 27
#define button3 26
#define button4 25
#define pin_signal 15
#define relay_in1
#define relay_in2
#define relay_in3

bool sendFlag = false; 
bool data_received = false;
int sinyal_masuk;

// variable for storing the potentiometer value
uint8_t value_transmit[8] = {00, 00, 00, 00, 00, 00, 00, 00};


/////////////////////////////////////////////////////////////////////////////////////////////////////////////


void twai_setup_and_install(){
    //Initialize configuration structures using macro initializers
    twai_general_config_t g_config = {
        .mode = TWAI_MODE_NORMAL,
        .tx_io = GPIO_NUM_22,
        .rx_io = GPIO_NUM_21,
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


/////////////////////////////////////////////////////////////////////////////////////////////////////////////


void new_message(twai_message_t *message, uint32_t id, uint8_t dlc, uint8_t *data){ 
    message->flags = TWAI_MSG_FLAG_EXTD;
    message->identifier = id;
    message->data_length_code = dlc;
    for (int i = 0; i < dlc; i++) {
        message->data[i] = data[i];
    }
    
    printf("\nMessage created\nID: %x DLC: %d Data:\t", message->identifier, message->data_length_code);
    
    for (int i = 0; i < message->data_length_code; i++) {
        printf("%d\t", message->data[i]);
    }
    
    printf("\n");
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////


void transmit_message(twai_message_t *message){
    if (twai_transmit(message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        printf("Message queued for transmission\n");
    } else {
        printf("Failed to send message\n");
    }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////


void receive_message(twai_message_t *message){
    if (twai_receive(message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        
        //printf("Message received:\n");
        //char ID = message->identifier;
        //printf(ID);
        // switch(message->identifier){
        //   case 14FEF121:
        //     printf("Glitched");
        //     break;          
        // }
        
        printf("\n");
        printf("ID: %x DLC: %d Data: \t", message->identifier, message->data_length_code);
        
        for (int i = 0; i < message->data_length_code; i++) {
            //(message->extd)?printf("Extended ID"):printf("Standard ID");
        
            printf("%02x\t", message->data[i]);
            
            if (message->data_length_code == 8 &&
                    message->data[0] == 0x00 && message->data[1] == 0x00 &&
                    message->data[2] == 0xff && message->data[3] == 0x0f &&
                    message->data[4] == 0x00 && message->data[5] == 0x00 &&
                    message->data[6] == 0x00 && message->data[7] == 0x00) {
                    printf("Out of range\n");
            }
        }
    } else {
        printf("Failed to receive message\n");
    }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////


void receive_message_for_send(twai_message_t *message){
    if (twai_receive(message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        printf("Message received:\n");
        char ID = message->identifier;
        printf("%x", ID);
        
        value_transmit[2] = message->data[2];
        value_transmit[3] = message->data[3];
        new_message(message, 0x18FEF121, 8, value_transmit);
        transmit_message(message); 
    } else {
        printf("Failed to receive message\n");
    }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() {
  Serial.begin(115200);
  
  twai_setup_and_install();
  
  pinMode(button1, INPUT);  
  pinMode(button2, INPUT);
  pinMode(button3, INPUT); 
  pinMode(button4, INPUT);
  pinMode(pin_signal, INPUT_PULLDOWN);

  delay(1000);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////


void loop() {
  sinyal_masuk = digitalRead(pin_signal);

  if (sinyal_masuk == HIGH && !data_received){
    Serial.println("CAN BUS TERHUBUNG!");
    data_received = true;
    
    twai_message_t message;
    twai_message_t message1;
  
    if (digitalRead(button1) == HIGH){
      Serial.println("send data");
      receive_message_for_send(&message);
      sendFlag = false;  
    } else{
      sendFlag = true;
    }
    
    // Receive the message from the queue
    receive_message(&message);
    // Wait for 1 second
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  } 
  else if (sinyal_masuk == LOW && data_received){
    Serial.println("DATA TIDAK MASUK!");
    data_received = false;
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
void buttonClickHandler() {
 sendFlag = true;
}
*/
