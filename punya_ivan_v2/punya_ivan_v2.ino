#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
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

// Replace with your network credentials
const char* ssid = "TP-Link_75B0";
const char* password = "69772436";

// IP address of the power supply
const char* powerSupplyIP = "192.168.0.10"; // Replace with the actual IP address
const uint16_t port = 5025; // Default SCPI port

// Set static IP address for the ESP32
IPAddress local_IP(192, 168, 0, 20); // Replace with your desired IP address
IPAddress gateway(192, 168, 0, 1); // Replace with your network gateway
IPAddress subnet(255, 255, 255, 0); // Replace with your network subnet mask
IPAddress primaryDNS(8, 8, 8, 8); // Optional: set a primary DNS server
IPAddress secondaryDNS(8, 8, 4, 4); // Optional: set a secondary DNS server

AsyncWebServer server(80);

WiFiClient client;

// create variable for mocking pin
const int pin1 = 2;
const int pin2 = 3;
const int pin3 = 4;
const int pin4 = 5;

// creare variable to read data from power supply
int powerSupplyMaxCurrent;
int powerSupplyMaxVoltage;

// from ev to dc station
twai_message_t id_transmit_102; twai_message_t id_receive_102;
twai_message_t id_transmit_101;
twai_message_t id_transmit_100;

// from dc station to ev 
twai_message_t id_receive_109; twai_message_t id_transmit_109;
twai_message_t id_receive_108; twai_message_t id_transmit_108;

// id 100
uint8_t maximum_battery_voltage; uint8_t maximum_battery_voltage_msb; uint8_t maximum_battery_voltage_lsb;
uint8_t constant_of_charging_rate_indication;

// id 101
uint8_t rated_capacity_of_battery; uint8_t rated_capacity_of_battery_msb; uint8_t rated_capacity_of_battery_lsb;
uint8_t maximum_charging_time; uint8_t minutes; uint8_t seconds; 
uint8_t estimated_charging_time;

// id 102
uint8_t supportedAppProtocol_Req;   // control protocol number
uint8_t target_battery_voltage; uint8_t target_battery_voltage_msb; uint8_t target_battery_voltage_lsb;
uint8_t charging_current_request;
uint8_t charging_rate;
uint8_t vehicle_charging_enable = 0;
uint8_t vehicle_shift_lever_position = 0;
uint8_t charging_system_fault = 0;
uint8_t vehicle_status = 1;
uint8_t normal_stop_request_before_charging = 0;
uint8_t battery_overvoltage = 0;
uint8_t battery_undervoltage = 0;
uint8_t battery_current_deviation_error = 0;
uint8_t high_battery_temperature = 0;
uint8_t battery_voltage_deviation_error = 0;

// id 108
uint8_t available_output_voltage; uint8_t available_output_voltage_msb; uint8_t available_output_voltage_lsb;
uint8_t available_output_current;
uint8_t charging_power;
uint8_t threshold_voltage;

// id 109
uint8_t case_109_5;
uint8_t supportedAppProtocol_Res;   // control protocol number
uint8_t output_voltage;
uint8_t output_current;
uint8_t remaining_charging_time;
uint8_t station_status;
uint8_t station_malfunction;
uint8_t vehicle_connector_lock;
uint8_t battery_incompatibility;
uint8_t charging_system_malfunction;
uint8_t charger_stop_control;

// Other variable send to Charger-Off-board
uint8_t Status_for_id_100[8] = {00, 00, 00, 00, 00, 00, 00, 00};
uint8_t Status_for_id_101[8] = {00, 00, 00, 00, 00, 00, 00, 00};
uint8_t Status_for_id_102[8] = {00, 00, 00, 00, 00, 00, 00, 00};
uint8_t Status_for_id_108[8] = {00, 00, 00, 00, 00, 00, 00, 00};
uint8_t Status_for_id_109[8] = {00, 00, 00, 00, 00, 00, 00, 00};

// variable to be validate
uint8_t validate_status_for_id_102[8] = {25, 00, 00, 00, 00, 00, 00, 00};

// counting for loop in each core  
long int prev_count_millis = millis();
long int prev_count_millis2 = millis();
long int prev_count_millis3 = millis();
long int prev_count_millis4 = millis();
long int prev_count_millis5 = millis();
long int prev_count_millis6 = millis();
long int prev_count_millis7 = millis();

// Global variables to track condition status
bool receiver_verification_status = true;
bool transmit_verification_status = true;
bool isConnected = false;
bool temperature_received = false;
bool voltage_current_received = false;
bool start_exchange = false;

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

void message_transfer(twai_message_t *message, uint32_t id, uint8_t dlc, uint8_t *data){ 
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
  transmit_message(message);
}

void transmit_message(twai_message_t *message){
  if (twai_transmit(message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    printf("Message queued for transmission\n");
  } else {
    printf("Failed to send message\n");
  }
}

/**
Create function for implemented can bus communication protocol
*/
void identification_chargecontroller_chargeroffboard(void * argument){

  for(;;){
    long int current_count_millis5 = millis();
      
    if(current_count_millis5 - prev_count_millis5 >= 1000){  
      // create id message for handshaking process
      if(!isConnected){
      Status_for_id_109[0] = 50;
      message_transfer(&id_transmit_109, 0x109, 8, Status_for_id_109);
      }

      // check for response
      if(twai_receive(&id_receive_102, pdMS_TO_TICKS(1000)) == ESP_OK){ 
        if(id_receive_102.identifier == 0x102){
          printf("\nID: %X DLC: %d \nData ID 102: \t", id_receive_102.identifier, id_receive_102.data_length_code);

          for(int i=0; i < id_receive_102.data_length_code; i++){
            
            printf("%d\t", id_receive_102.data[i]);

            if (id_receive_102.data[i] != validate_status_for_id_102[i]){
              receiver_verification_status = false;
              isConnected = false;
              printf("\nData charger off-board not match!");
              break;
            } else{
              receiver_verification_status = true;
              transmit_verification_status = true;
              isConnected = true;
              // assign status id from vehicle
              Status_for_id_102[0] = id_receive_102.data[0];
            } 
          }

        }
      }

    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// function for insitialize data exchange for transmitting value to ev
void exchange_data_communication_initialization_transmit(void * argument){
  // check if initialization complete at first
  if(receiver_verification_status == true && transmit_verification_status == true){
    // set start exchange to be true for starting data exchange
    start_exchange = true;

    // loop while data exchange periode running
    while(start_exchange){
      long int current_count_millis6 = millis();   

      if(current_count_millis6 - prev_count_millis6 >= 1000){
        //sending data
        // assign variabel default value
        powerSupplyMaxCurrent = 0;
        powerSupplyMaxVoltage = 0;

        // get data from power supply
        String getValueMaxVoltage = sendCommand("SOUR:VOLT:MAX?");
        vTaskDelay(500);
        String getValueMaxCurrent = sendCommand("SOUR:CURR:MAX?");

        // convert value from string to integer for each variable
        powerSupplyMaxVoltage = getValueMaxVoltage.toInt();
        powerSupplyMaxCurrent = getValueMaxCurrent.toInt();

        // assign to memory byte
        Status_for_id_108[3] = powerSupplyMaxCurrent;

        // Convert the voltage to an integer representation (e.g., 12.34 -> 1234)
        uint16_t voltageInt = (uint16_t)(powerSupplyMaxVoltage * 100);

        Status_for_id_108[1] = (uint8_t)(voltageInt >> 8);   // High byte
        Status_for_id_108[2] = (uint8_t)(voltageInt & 0xFF); // Low byte

        // send data
        message_transfer(&id_transmit_108, 0x108, 8, Status_for_id_108);

        // give some delay
        vTaskDelay(200);

        // read charging station condition
        // condition are created for mocking
        station_status = (digitalRead(pin1) == HIGH) ? 1 : 0;
        station_malfunction = (digitalRead(pin1) == HIGH) ? 1 : 0;
        vehicle_connector_lock = (digitalRead(pin1) == HIGH) ? 1 : 0;
        battery_incompatibility = (digitalRead(pin1) == HIGH) ? 1 : 0;
        charging_system_malfunction = (digitalRead(pin1) == HIGH) ? 1 : 0;
        charger_stop_control = (digitalRead(pin1) == HIGH) ? 1 : 0;

        // Create an array to hold the pin states
        int pinStates[6];

        // Read the state of each pin and store it in the array
        pinStates[0] = station_status;
        pinStates[1] = station_malfunction;
        pinStates[2] = vehicle_connector_lock;
        pinStates[3] = battery_incompatibility;
        pinStates[4] = charging_system_malfunction;
        pinStates[5] = charger_stop_control;

        // Combine the pin states into a single integer
        int decimalValue = 0;
        for (int i = 0; i < 4; i++) {
          decimalValue |= (pinStates[i] << i);
        }

        Status_for_id_109[5] = decimalValue;
        message_transfer(&id_transmit_109, 0x109, 8, Status_for_id_109);
      }

      // add computation delay
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}