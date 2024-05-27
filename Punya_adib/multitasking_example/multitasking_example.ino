#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdio.h"
#include "stdlib.h"
#include "driver/twai.h"
#include <SD.h>
#include "SPI.h"
#include "FS.h"
#include "BluetoothSerial.h"
#define RX_PIN 21
#define TX_PIN 22

TaskHandle_t readStatusHandle = NULL;
TaskHandle_t receiveMessageHandle = NULL;
TaskHandle_t inverterStatusHandle = NULL;
TaskHandle_t pdu123Handle = NULL;
TaskHandle_t dcdcStateHandle = NULL;

twai_message_t message;
void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);
  File file = fs.open(path, FILE_WRITE);
  
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  
  file.close();
}

void twai_setup_and_install_for_send(){
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
void read_inverter_status(void *arg){
  while(1){
  if(twai_receive(&message, pdMS_TO_TICKS(1000)) == ESP_OK){
    if(message.identifier == 0x0CF00402){
        printf("\nID Inverter: %x", message.identifier);        
        File file = SD.open("/inverter_status.txt", FILE_APPEND);
        if (!file) {
          Serial.println("File not exist yet");
        }
        else{
          Serial.println("File inverter status already exist");
        }
        uint8_t status = message.data[0];
        uint8_t mock_value[8] = {02, 43, status, 00, 00, 00, 00, 00};
        for(int i = 0; i < message.data_length_code; i++){
          printf("%x\t", mock_value[i]);
          //  // Append data to the file
          if (file.print(String(mock_value[i]))) {
            Serial.println("Message inverter appended");
          } else {
            Serial.println("Append failed");
          }
        }
        if (file.print("\n")) {
            Serial.println("Enter appended");
        } else {
            Serial.println("Append failed");
        }
        printf("\n");
  }
  }
  printf("\nWaktu kemakan buat inverter: %d",millis());
  }
}

void idmt_receive(void *message){
  while(1){
  if(twai_receive(&message, pdMS_TO_TICKS(1000)) == ESP_OK){
    if(message.identifier == 0x18DB33F1){
        printf("\nPGN ID: %x", message.identifier);        
        uint8_t currentValue = message.data[1];
        float realcurrent = currentValue;
        printf("Current Value: %.0f A\n", realcurrent);
        loat Ir_mul = BatteryCurrent / Is; 
    
        if (Ir_mul <= 2 ){
          Serial.println("Battery OK");
        }
        else if (Ir_mul > 2 && Ir_mul < 4){
          Serial.println("IDMT Sequence Start");
          double trip_duration = idmt_veryinverse (BatteryCurrent);
          tripping(trip_duration);
        }
        else {
          Serial.println("Instanteous Trip");
          RelayON();
        }
        delay(100);
      }
  }
}


void read_pdu_state1(void *arg){
  while(1){
  if(twai_receive(&message, pdMS_TO_TICKS(1000)) == ESP_OK){
    switch(message.identifier){
      case 0x10F51402:
        printf("\nID PDU1: %x", message.identifier);
        File file = SD.open("/pdu_status.txt", FILE_APPEND);
        if (!file) {
          Serial.println("File does not exist yet");
        }
        else{
          Serial.println("File pdu status already exist");
        }
        // uint8_t status_bms_msb = message->data[1];
        // uint8_t status_bms_lsb = message->data[0];
        for(int i = 0; i < message.data_length_code; i++){
          printf("%x\t", message.data[i]);
          //  // Append data to the file
           if (file.print(String(message.data[i]))) {
              Serial.println("Message pdu1 appended");
           } else {
              Serial.println("Append failed");
           }
        }
        if (file.print("\nEnter")) {
              Serial.println("Enter appended");
        } else {
              Serial.println("Append failed");
        }
        printf("\n");
  }
  }
  printf("\nWaktu kemakan buat pdu1: %d", millis());
  }  
}

void read_pdu_state2(void *arg){
  while(1){
  if(twai_receive(&message, pdMS_TO_TICKS(1000)) == ESP_OK){
    switch(message.identifier){
      case 0x18FE8D02:
        printf("\nID PDU2: %x", message.identifier);
        File file = SD.open("/pdu_status.txt", FILE_APPEND);
        if(!file){
          Serial.println("File does not exist yet");
        }
        else{
          Serial.println("File pdu state already exist");
        }
        uint8_t dcdc_state = message.data[0];
        uint8_t mock_value[8] = {01, 43, dcdc_state, 00, 00, 00, 00, 00};
        for(int i = 0; i < message.data_length_code; i++){
          printf("%x\t", mock_value[i]);
          //  // Append data to the file
           if (file.print(String(mock_value[i]))) {
              Serial.println("Message pdu2 appended");
           } else {
              Serial.println("Append failed");
           }
        }
        if (file.print("\nEnter")) {
              Serial.println("Enter appended");
        } else {
              Serial.println("Append failed");
        }
        printf("\n");
  }
  }
  printf("\nWaktu kemakan buat pdu2: %d", millis());
  }
}

void read_pdu_state3(void *arg){
  while(1){
  if(twai_receive(&message, pdMS_TO_TICKS(1000)) == ESP_OK){
    switch(message.identifier){
      case 0x18FE8D03:
        printf("\nID PDU3: %x", message.identifier);
        File file = SD.open("/pdu_state.txt", FILE_APPEND);
        if(!file){
          Serial.println("File does not exist");
        }
        else{
          Serial.println("File pdu state already exist");
        }
        uint8_t lv_state = message.data[0];
        uint8_t mock_value[8] = {01, 43, 00, lv_state, 00, 00, 00, 00};
        for(int i = 0; i < message.data_length_code; i++){
          printf("%x\t", mock_value[i]);
          //  // Append data to the file
           if (file.print(String(mock_value[i]))) {
              Serial.println("Message pdu3 appended");
           } else {
              Serial.println("Append failed");
           }
        }
        if (file.print("\nEnter")) {
              Serial.println("Enter appended");
        } else {
              Serial.println("Append failed");
        }
        printf("\n"); 
  }
  }
  printf("\nWaktu kemakan buat pdu3: %d", millis());
  }
}

void read_dcdc_state(void *arg){
  while(1){
  if(twai_receive(&message, pdMS_TO_TICKS(1000)) == ESP_OK){
    switch(message.identifier){
      case 0x18F11401:
        printf("\nID DCDC: %x", message.identifier);
        File file = SD.open("/dcdc_state.txt", FILE_APPEND);
        if(!file){
          Serial.println("File does not exist yet");
        }
        else{
          Serial.println("File dcdc state already exist");          
        }
        for(int i = 0; i < message.data_length_code; i++){
          printf("%x\t", message.data[i]);
          //  // Append data to the file
           if (file.print(String(message.data[i]))) {
              Serial.println("Message dcdc appended");
           } else {
              Serial.println("Append failed");
           }
        }
        if (file.print("\n")) {
              Serial.println("Enter appended");
        } else {
              Serial.println("Append failed");
        }
        printf("\n");  
  }
  }
  printf("\nWaktu abis buat dcdc: %d", millis());
  }
}
void setup(){
  Serial.begin(115200);
  twai_setup_and_install_for_send();
  if(!SD.begin(5)){
    Serial.println("Card Mount Failed");
    // return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    // return;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  File file = SD.open("/test_log.txt");
  if(!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/test_log.txt", "test\r\n");
  }
  else {
    Serial.println("File already exists");  
  }
  
  file.close();

  file  = SD.open("/dcdc_state.txt");
  if(!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/dcdc_state.txt", "test\r\n");
  }
  else {
    Serial.println("File dcdc state already exists");  
  }
  
  file.close();

  file = SD.open("/pdu_state.txt", FILE_APPEND);
  if(!file){
    Serial.println("File does not exist yet");
    writeFile(SD, "/pdu_state.txt", "test\r\n");
  }
  else{
    Serial.println("File pdu state already exist");
  }
  file.close();

  file = SD.open("/bms_status.txt", FILE_APPEND);
  if(!file){
    Serial.println("File does not exist yet");
    writeFile(SD, "/bms_status.txt", "test\r\n");
  }
  else{
    Serial.println("File bms state already exist");
  }
  file.close();

  file = SD.open("/inverter_state.txt", FILE_APPEND);
  if(!file){
    Serial.println("File does not exist yet");
    writeFile(SD, "/inverter_state.txt", "test\r\n");
  }
  else{
    Serial.println("File inverter state already exist");
  }
  file.close();
  xTaskCreate(read_inverter_status, "inverterStatus", 3000, NULL, 3, &inverterStatusHandle);
  xTaskCreate(read_pdu_state1, "pduState1", 3000, NULL, 2, &pdu123Handle);
  xTaskCreatePinnedToCore(read_pdu_state2, "pduState2", 3000, NULL, 2, &pdu123Handle, 1);
  xTaskCreatePinnedToCore(read_pdu_state3, "pduState3", 3000, NULL, 1, &pdu123Handle, 1);
  xTaskCreate(read_dcdc_state, "dcdcState", 3000, NULL, 1, &dcdcStateHandle);
  vTaskDelete(NULL);

  //inget, priority makin tinggi, resource pool ke tugas itu makin banyak
}