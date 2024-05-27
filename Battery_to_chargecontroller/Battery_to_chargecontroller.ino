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

#define Voltage_Pin 27
#define Current_Pin 26
#define Temp_Pin 25

long int prevMillis = 0;
long int prevMillis2 = 0;

twai_message_t message_battery_current_voltage;
twai_message_t message_battery_temperature_value;

// variable for storing the potentiometer value
uint8_t Voltage_Current_value[8] = {00, 00, 00, 00, 00, 00, 00, 00};
uint8_t Temperature_value[8] = {00, 00, 00, 00, 00, 00, 00, 00};

uint8_t I_potensio = 0;
uint8_t V_potensio = 0;
uint8_t T_potensio = 0;


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
  Serial.begin(115200); 
  twai_setup_and_install(); 
  delay(1000);
}


void transmit_message(twai_message_t *message){
  if (twai_transmit(message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    printf("Message queued for transmission\n");
  } else {
    printf("Failed to send message");
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
    printf("Failed to send message");
  }
}


void loop() {
  long int currentMillis = millis();
  long int currentMillis2 = millis();

  
  I_potensio = analogRead(Current_Pin);
  V_potensio = analogRead(Voltage_Pin);
  T_potensio = analogRead(Temp_Pin);

  // Extract the most significant and least significant bytes
  uint8_t current_msb = (I_potensio >> 8) & 0xFF;
  uint8_t current_lsb = I_potensio & 0xFF;

  uint8_t voltage_msb = (V_potensio >> 8) & 0xFF;
  uint8_t voltage_lsb = (V_potensio & 0xFF);

  uint8_t temp = T_potensio & 0xFF;

  Voltage_Current_value[2] = current_msb;
  Voltage_Current_value[1] = current_lsb;
  Voltage_Current_value[4] = voltage_msb;
  Voltage_Current_value[3] = voltage_lsb;

  Temperature_value[1] = temp;

  if(currentMillis - prevMillis >= 1000){
    new_message(&message_battery_current_voltage, 0x18FD15FE, 8, Voltage_Current_value);
    // receive_message(&message1);
    prevMillis = currentMillis;
  }

  if(currentMillis2 - prevMillis2 >= 1000){
    new_message(&message_battery_temperature_value, 0x18FE50FE, 8, Temperature_value);
    // receive_message(&message1);
    prevMillis2 = currentMillis2;
  }
}
