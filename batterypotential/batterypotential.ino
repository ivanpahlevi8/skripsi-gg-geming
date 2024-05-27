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
#define Current_Pin 4
// #define Voltage_Pin ...

long int prevMillis = 0;

// variable for storing the potentiometer value
uint8_t Current_value[8] = {00, 00, 00, 00, 00, 00, 00, 00};
int I_potensio = 0;
int V_potensio = 0;


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

void setup() {
  twai_setup_and_install(); 
  Serial.begin(115200); 
  delay(1000);
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
  printf("\n");
}


void transmit_message(twai_message_t *message){
  if (twai_transmit(message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    printf("Message queued for transmission\n");
  } else {
    printf("Failed to send message\n");
  }
}


void receive_message(twai_message_t *message){
  if (twai_receive(message, pdMS_TO_TICKS(1000)) == ESP_OK) {          
    printf("\n");
    printf("ID: %x DLC: %d Data: \t", message->identifier, message->data_length_code);
    
    for (int i = 0; i < message->data_length_code; i++) {
      printf("%02x\t", message->data[i]);
    }

  } else {
    printf("Failed to receive message\n");
  }
}


void loop() {
  // Reading potentiometer value
  twai_message_t message;
  twai_message_t message1;
  long int currentMillis = millis();
  
  I_potensio = analogRead(Current_Pin);
  // V_potensio = analogRead(Voltage_Pin);
  // int arus_baterai = map(I_potensio, 0, 4095, 0, 40);

  // Extract the most significant and least significant bytes
  uint8_t msb = (I_potensio >> 8) & 0xFF;
  uint8_t lsb = I_potensio & 0xFF;

  Current_value[3] = msb;
  Current_value[2] = lsb;


  if(currentMillis - prevMillis >= 1000){
    new_message(&message, 0x18FCD121, 8, Current_value);
    receive_message(&message1);
    prevMillis = currentMillis;
  }
  // receive_message(&message1);
  // transmit_message(&message);

  // vTaskDelay(1000 / portTICK_PERIOD_MS);
}
