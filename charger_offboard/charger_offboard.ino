#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdio.h"
#include "stdlib.h"
#include "driver/twai.h"
#include "SD.h"
#include "SPI.h"
#include "FS.h"

#define RX_PIN 21
#define TX_PIN 22


#define charge_lock 15
#define sens_current 26
#define sens_voltage 32
#define can_bus_connect 27
#define leakage_current_sens 14
#define precharge_1 25
#define precharge_2 4


long int prev_count_millis = millis();
long int prev_count_millis2 = millis();


twai_message_t message_for_transmit;
twai_message_t message_for_receive;

// bool message_transmitted = false;


void twai_setup_and_install(){
  //Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
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


void transmit_message(twai_message_t *message){
  if (twai_transmit(message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    printf("Message queued for transmission\n");
  } else {
    printf("Failed to send message\n");
  }
}


void new_message(twai_message_t *message, uint32_t id, uint8_t dlc, uint8_t *data){    
  message->flags = TWAI_MSG_FLAG_EXTD;
  message->identifier = id;
  message->data_length_code = dlc;
  
  for (int i = 0; i < dlc; i++) {
    message->data[i] = data[i];
  }
  printf("\nID: %x DLC: %d Data:\t", message->identifier, message->data_length_code);
  for (int i = 0; i < message->data_length_code; i++) {
    printf("%d\t", message->data[i]);
  }
  if (twai_transmit(message, pdMS_TO_TICKS(1000)) == ESP_OK) {
  } else {
    printf("Failed to send message\n");
  }
}




void setup() {
  printf("================================  Program Start!  ================================\n");
  twai_setup_and_install(); 
  Serial.begin(115200); 
  delay(1000);


void loop() {

  // if(currentMillis - prevMillis >= 1000){
  //   new_message(&message_battery_current_voltage, 0x18FD15FE, 8, Voltage_Current_value);
  //   new_message(&message_battery_temperature_value, 0x18FE50FE, 8, Temperature_value);
  //   receive_message(&message1);
  //   prevMillis = currentMillis;
  // }

}

