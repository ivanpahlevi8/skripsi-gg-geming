#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdio.h"
#include "stdlib.h"
#include "driver/twai.h"
#include "SD.h"
#include "SPI.h"
#include "FS.h"

uint8_t data_sent[8] = {04, 41, 12, 00, 00, 00, 00, 00};

#define RX_PIN 21
#define TX_PIN 22
#define potPin 4
// variable for storing the potentiometer value
int potValue = 0;


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

void loop() {
  // Reading potentiometer value

  twai_message_t message;

  static uint32_t CANTimeout = millis();


  potValue = analogRead(potPin);
  //int BatteryCurrent = potValue / 40.95;
  int BatteryCurrent = map(potValue, 0, 4095, 0, 40);
  int BatteryTemp = map(potValue, 0, 4095, 20, 80);

  Serial.println("Battery Current =" + String(BatteryCurrent));
  
  uint8_t SPN_114 = BatteryCurrent & 0xFF;
  uint8_t SPN_1800 = BatteryTemp & 0xFF;

  Serial.println("SPN 114 = " + String(SPN_114));

  Serial.println("Battery Temperature =" + String(BatteryCurrent));
  Serial.println("SPN 1800 = " + String(SPN_1800));
  // Store the values in your array

  if (millis() - CANTimeout >= 100) {
    message.identifier = 0x10261022;
    message.extd = 1;
    message.data_length_code = 8; // Length
    message.data[0] = 0x9 ; // Fan Speed times 0.4%
    message.data[1] = BatteryCurrent; // Battery Current 115
    message.data[2] = 0xAA; // Alternator Current 115
    message.data[3] = 0xAA; // Charging System Potential (Voltage) 167
    message.data[4] = 0xAA; // Charging System Potential (Voltage) 167
    message.data[5] = BatteryTemp; // Battery Temperature / 1800
    message.data[6] = BatteryCurrent; // Battery Potential / Power Input 1 168
    message.data[7] = 0;  // Battery Potential / Power Input 1 168
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK)  {
    } else {
      printf("Failed to send message\n");
    }
  }

  vTaskDelay(1000/ portTICK_PERIOD_MS);
}
