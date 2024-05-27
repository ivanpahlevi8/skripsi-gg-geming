#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdio.h"
#include "stdlib.h"
#include "driver/twai.h"
#include "SD.h"
#include "SPI.h"
#include "FS.h"

#define pin_signal 15
#define voltagesens1 13
#define voltagesens2 14
#define firstflag 27
#define secondflag 26
#define thirdflag 25

bool out_of_range = true; 
bool sendFlag = false; 
bool condition_met_can_h = false;
bool condition_met_can_l = false;
bool condition_voltage = false;
//bool data_received = false;
int sinyal_masuk;

int voltage_sens1_offset = 90;
int voltage_sens2_offset = 90;
float voltage_conversion_factor = 5.0 / 1023.0;            // Calibration factors for converting sensor readings to voltage

// variable for storing the potentiometer value
uint8_t value_transmit[8] = {00, 00, 00, 00, 00, 00, 00, 00};
const unsigned char full_range[8] = {0x00, 0x00, 0xff, 0x0f, 0x00, 0x00, 0x00, 0x00};
twai_message_t test_message;
twai_message_t message_sender;


#define RX_PIN 21
#define TX_PIN 22

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

  if(twai_transmit(message, pdMS_TO_TICKS(1000)) == ESP_OK){
    Serial.println("MEssage sent!");
  };    
}

/*
void transmit_message(twai_message_t *message){
    if (twai_transmit(message, pdMS_TO_TICKS(1000)) == ESP_OK) {
        printf("Message queued for transmission\n");
    } else {
        printf("Failed to send message\n");
    }
}
*/

void receive_message(twai_message_t *message){
  if (twai_receive(message, pdMS_TO_TICKS(1000)) == ESP_OK) {  
    printf("\n");
    printf("ID: %x DLC: %d Data: \t", message->identifier, message->data_length_code);

    for (int i = 0; i < message->data_length_code; i++) {
      printf("%02x\t", message->data[i]);
    }

    // if (message->data_length_code == 8 &&
    //   message->data[0] == 0x00 && message->data[1] == 0x00 &&
    //   message->data[2] == 0xff && message->data[3] == 0x0f &&
    //   message->data[4] == 0x00 && message->data[5] == 0x00 &&
    //   message->data[6] == 0x00 && message->data[7] == 0x00){
    //   printf("Out of range\n");    
    // }  
  } else {
    printf("Failed to receive message\n");
  }
}


void send_back(twai_message_t *message){
  if (twai_receive(message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    char ID = message->identifier;

    value_transmit[2] = message->data[2];
    value_transmit[3] = message->data[3];
    
    new_message(message, 0x18FEF121, 8, value_transmit);
    // transmit_message(message); 
  
  } else {
    printf("Failed to receive message\n");
  }
}

void voltage_sensor(){
  int sens_voltage1 = analogRead(voltagesens1);
  int sens_voltage2 = analogRead(voltagesens2);

  sens_voltage1 += voltage_sens1_offset;
  sens_voltage2 += voltage_sens2_offset;  

  double voltage1 = sens_voltage1 * voltage_conversion_factor;
  double voltage2 = sens_voltage2 * voltage_conversion_factor;

  while(condition_voltage != true){ 
    Serial.print("Voltage CAN H: ");
    Serial.print(voltage1);
    Serial.println("V");

    Serial.print("Voltage CAN L: ");
    Serial.print(voltage2);
    Serial.println("V\n");

    // Check the condition for CAN H
    if (voltage1 >= 1.25 && voltage1 <= 1.75) {
      if (!condition_met_can_h) {
        Serial.println("Checking CAN H Complete");
        condition_met_can_h = true; 
      }
    } else {
      condition_met_can_h = false; 
      Serial.println("Checking CAN H Not Complete");
    }

    // Check the condition for CAN L
    if (voltage2 >= 1.25 && voltage2 <= 1.75) {
      if (!condition_met_can_l) {
        Serial.println("Checking CAN L Complete");
        condition_met_can_l = true; 
      }
    } else {
      condition_met_can_l = false; 
      Serial.println("Checking CAN L Not Complete");
    }
    
    if (condition_met_can_h && condition_met_can_l){
      condition_voltage = true;
    }    
    
    Serial.print("\n");
    delay(1000);

  }
}

void setup() {
  twai_setup_and_install();
  Serial.begin(115200);

  pinMode(pin_signal, INPUT_PULLDOWN);
  pinMode(firstflag, OUTPUT);
  pinMode(secondflag, OUTPUT);
  pinMode(thirdflag, OUTPUT);
 
  delay(1000);
}


void loop() {
  // sinyal_masuk = digitalRead(pin_signal);
  
  // voltage_sensor();

  // switch (sinyal_masuk) {
  //   case HIGH:
  //     if (condition_met_can_h && condition_met_can_l) {
  //       digitalWrite(firstflag, HIGH);

  //       twai_message_t message;
  //       twai_message_t message1;
        
  //       send_back(&message);
  //       receive_message(&message);
        
  //       vTaskDelay(1000 / portTICK_PERIOD_MS);
  //     }
  //     break;

  //   case LOW:
  //     Serial.println("DATA TIDAK MASUK!");
  //     digitalWrite(firstflag, LOW);
  //     delay(2000);
  //     break;

  //   default:
  //     break;
  // }
  receive_message(&test_message);
  new_message(&message_sender, 0x12345, 8, value_transmit);

}
