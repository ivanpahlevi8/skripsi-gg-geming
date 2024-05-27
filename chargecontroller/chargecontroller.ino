#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdio.h"
#include "stdlib.h"
#include "driver/twai.h"
#include "SD.h"
#include "SPI.h"
#include "FS.h"

#define locking_pin 15
#define firstflag 27
#define secondflag 26
#define thirdflag 25

long int prevMillis = 0;

bool out_of_range = true; 
bool sendFlag = false; 
int sinyal_masuk;


// variable for storing the potentiometer value
uint8_t value_transmit[8] = {00, 00, 00, 00, 00, 00, 00, 00};
const unsigned char full_range[8] = {0x00, 0x00, 0xff, 0x0f, 0x00, 0x00, 0x00, 0x00};


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

  transmit_message(message);    
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
    digitalWrite(firstflag, HIGH);
   
  Serial.print("\nID: ");
  Serial.print(message->identifier, HEX);
  Serial.print(" DLC: ");
  Serial.print(message->data_length_code);
  Serial.print(" Data: \t");

  for (int i = 0; i < message->data_length_code; i++) {
    Serial.print(message->data[i], HEX);
    Serial.print("\t");
  }

    if (message->data_length_code == 8 &&
        message->data[0] == 0x00 && message->data[1] == 0x00 &&
        message->data[2] == 0xff && message->data[3] == 0x0f &&
        message->data[4] == 0x00 && message->data[5] == 0x00 &&
        message->data[6] == 0x00 && message->data[7] == 0x00){
      Serial.print("Out of range");    
    }
  send_back(message); 

  } else {
    Serial.println("DATA TIDAK MASUK!");
    digitalWrite(firstflag, LOW);
  }

}

void send_back(twai_message_t *message){
  int ID = message->identifier;
  // message->identifier = 0x18FCD123;

  value_transmit[2] = message->data[2];
  value_transmit[3] = message->data[3];
    
  new_message(message, ID, 8, value_transmit);
  transmit_message(message); 
}

// void relay_lock{
//   if(firstflag && secondflag && thirdflag){
//     digitalWrite(locking_pin, HIGH);
//   } else{
//     diigitalWrite(locking_pin, LOW);
//   }
// }

void setup() {
  twai_setup_and_install();
  Serial.begin(115200);

  pinMode(firstflag, OUTPUT);
  pinMode(secondflag, OUTPUT);
  pinMode(thirdflag, OUTPUT);
  pinMode(locking_pin, OUTPUT);
 
  delay(1000);
}


void loop() {
  twai_message_t message;
  twai_message_t message1;
  long int currentMillis = millis();
  
  if(currentMillis - prevMillis >= 1000){
    receive_message(&message);
    // receive_message(&message_battery_current_voltage);
    // send_back(&message);    
    // send_back(&message1);
    prevMillis = currentMillis;
  }
        
  // vTaskDelay(1000 / portTICK_PERIOD_MS);

}
