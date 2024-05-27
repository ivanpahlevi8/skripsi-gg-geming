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
//#define FEMALE_RELAY  4
#define FEMALE_RELAY  14
#define LED_OUT 27

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

TaskHandle_t current_task, idmt_task, sd_task, temp_task, status_task;
SemaphoreHandle_t baton;
double  Is = 20;

unsigned long current_millis = 0;
unsigned long millis_previous_read_LED  = 0;
unsigned long LEDblink_interval=200;
int LEDblink_state = 0;

int trip_time = 0; //trip time countdown
unsigned long trip_countdown_start;
unsigned long millis_countdown_read;

int potValue = 0;
int relaycount = 3;
double second = 0;
long previousMillis = 0;

typedef struct Protection_Info {
  double current;
  double temp;
  double voltage = 0;

  double countdown_log = 0;
  double trip_duration = 0;
  double current_ratio = 0;
  double current_ratio_old = 0;
  double current_ratio_add = 0;

  uint16_t protection_status = 0;
  uint16_t usage_status = 0;
  uint8_t state = 0;
  uint8_t temp_status = 0;          // InverterCondition
} Protection_Info;

Protection_Info battery;

void RelayON() {
  Serial.println(">>>>>>>>>>>!-------------");
  for(int i = 0; i < relaycount; i++) {
    digitalWrite(FEMALE_RELAY, HIGH);    
  }
  digitalWrite(FEMALE_RELAY, HIGH);
  delay(5000);
  Serial.println("Tripped");
}


void read_current( void * parameter )
{
  for (;;) {
    long start = millis();
    if (twai_receive(&message, pdMS_TO_TICKS(1000)) == ESP_OK){
      if(message.identifier == 0x10261022){
        long start = millis();
        battery.current = message.data[6];
        printf("        Battery Current: %.0f A\n", battery.current);
        battery.temp = message.data[5];
        printf("        Battery Temperature: %.0f C\n", battery.temp);
      }
    }else{
      printf("        Failed to receive message\n");
    }
    digitalWrite(LED_OUT, 1);

    vTaskDelay(500/ portTICK_PERIOD_MS);
  }
}

double idmt_veryinverse (float current){
   double k = 80;
   double alpha = 8; 
   double TMS = 0.5; 
   //double Is = 40;
   double Ir = (current / Is);
   double duration = TMS * (k/(pow(Ir, alpha)-1));
   return duration;
}

String time_left(unsigned long millis_left){
  String result;
  int S;
  int S_decimal;
  S= (long)millis_left/1000;
  S_decimal = (long)millis_left-S*1000;
  result=(String)S+"."+S_decimal+ " s";
  return result;
}

void idmt_sequence( void * parameter )
{
  for (;;) {
    long start = millis();
    battery.trip_duration = 0;
    float C_Rate = battery.current / Is;

        if (C_Rate <= 1 ){
          Serial.println("Battery OK");
          battery.protection_status = 1;
        }
        else if (C_Rate > 1 && C_Rate < 2){
          Serial.println("IDMT Sequence Start");
          battery.trip_duration = idmt_veryinverse (battery.current);
          float current_rec = battery.current;
          int state = 0;
          
          battery.current_ratio = battery.current / battery.trip_duration;
          battery.current_ratio_add = battery.current_ratio_old + battery.current_ratio;
          battery.current_ratio_old = battery.current_ratio_add;

          Serial.println(battery.current_ratio_add);

          if(battery.current_ratio_add <= 75){
            battery.usage_status = 1;
          }else if(battery.current_ratio_add <= 150){
            battery.usage_status = 2;
          }else if(battery.current_ratio_add > 150){
            battery.usage_status = 3;
          }

          trip_countdown_start = millis();
          do{
            current_millis = millis();
            if (battery.trip_duration*1000 > (millis() - trip_countdown_start)){
            Serial.println(time_left(battery.trip_duration*1000-(millis()-trip_countdown_start)));
            battery.countdown_log = battery.trip_duration*1000-(millis()-trip_countdown_start);
            //delay(100);
            if(battery.current < current_rec){
              digitalWrite(LED_BUILTIN, LOW);
              //digitalWrite(LED_OUT, LOW);
              break;
            } 
            if(battery.current > current_rec + 1){
              digitalWrite(LED_BUILTIN, LOW);
              //digitalWrite(LED_OUT, LOW);
              break;
            }
            battery.protection_status = 2;               

            if (current_millis - millis_previous_read_LED >= LEDblink_interval){
            millis_previous_read_LED = current_millis;
            if (LEDblink_state == 0) LEDblink_state = 1;  else LEDblink_state = 0;
            digitalWrite(LED_BUILTIN, LEDblink_state);
            //digitalWrite(LED_OUT, LEDblink_state);
            }

            }
            else{
              battery.protection_status = 3; 
              RelayON();
              digitalWrite(FEMALE_RELAY, LOW);
              state = 1;      
            }
            
          }while(state == 0);
          battery.countdown_log = 0;
        }
        else {
          Serial.println("Instanteous Trip");
          battery.protection_status = 4;
          RelayON();
          digitalWrite(FEMALE_RELAY, LOW);     
        } 
        
        Serial.println(String(battery.current_ratio_add));
          if(battery.current_ratio_add > 10){
            battery.state++;

            if(battery.state = 1000){
            battery.current_ratio_add = 0;
            battery.state = 0;
            }
          }                   

    //battery.trip_duration = 0;
    Serial.print("This Task runs on Core : ");
    Serial.println(xPortGetCoreID());
    Serial.print("Time ");
    Serial.println(millis() - start);
    //realcurrent = 0;
    vTaskDelay(1000/ portTICK_PERIOD_MS);
  }
}

void temp_protection(void * parameter){
  for (;;){
    if (battery.temp > 60){

    }else if(battery.temp > 60){
      Serial.println("Temperature >60, Cooling On");
      //Cooling On
      //Check Temp
    }else if(battery.temp > 70){
      Serial.println("Temperature >70, Cooling On, Limit Current");
      //Cooling On
      //Limit Current
      //Check Temp
    }else if(battery.temp > 100){
      //Trip
      Serial.println("Temperature >100, Dangerous");
    }else{
      Serial.println("Temperature Normal");
    }
    vTaskDelay(1000/ portTICK_PERIOD_MS);
  }

}

void print_sd(void * parameter){
  for (;;) {
    File file = SD.open("/bat_log.txt", FILE_APPEND);
    if(!file){
      Serial.println("        File does not exist yet");
    }
    else{
      Serial.println("        File battery log already exist");
    }
    if (file.print(String(millis()) + "," + String(battery.current) + "," + String(battery.protection_status) + "," + String(battery.trip_duration) + ","+ String(battery.countdown_log) + ","+ String(battery.usage_status) + "\n")) {
      Serial.println("        Message appended");
    } else {
      Serial.println("        Append failed");
    }
  vTaskDelay(1000/ portTICK_PERIOD_MS);
  }
}

void send_status( void * parameter )
{
  for (;;) {
    long start = millis();
    
    if (millis() - start >= 100) {
    message.identifier = 0x0FF01;
    message.extd = 1;
    message.data_length_code = 8; // Length
    message.data[0] = battery.current ; // Battery Current
    message.data[1] = battery.trip_duration; // Trip Duration
    message.data[2] = battery.temp; // Battery Temperature
    message.data[3] = battery.temp_status; // Battery Temperature Status
    message.data[4] = battery.usage_status; // Battery Usage Status
    message.data[5] = battery.protection_status; // Battery Protection Status
    message.data[6] = battery.countdown_log; // Battery Countdown Log
    message.data[7] = 0xAA;  // End Frame
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK)  {
    } else {
      printf("Failed to send message\n");
    }
  }
    vTaskDelay(500/ portTICK_PERIOD_MS);
  }
}

// the setup function runs once when you press reset or power the board
void setup() {
  twai_setup_and_install();
  Serial.begin(115200);
  delay(1000);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(FEMALE_RELAY, OUTPUT);
  pinMode(LED_OUT, OUTPUT);



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

  File file = SD.open("/bat_log");
  if(!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/bat_log.txt", "Timestamp,Battery Current,Battery Protection Status,Trip Duration,Countdown,Usage Status\n");
    
  }
  else {
    Serial.println("File already exists");  
  }
  
  file.close();


  xTaskCreatePinnedToCore(
    read_current,
    "current_task",
    10000,
    NULL,
    1,
    &current_task,
    0);
  
  xTaskCreatePinnedToCore(
    send_status,
    "status_task",
    5000,
    NULL,
    2,
    &status_task,
    0);

  xTaskCreatePinnedToCore(
    print_sd,
    "sd_task",
    5000,
    NULL,
    3,
    &sd_task,
    0);    
    
  //delay(500); // needed to start-up task1

  xTaskCreatePinnedToCore(
    idmt_sequence,
    "idmt_task",
    10000,
    NULL,
    1,
    &idmt_task,
    1);
  
  xTaskCreatePinnedToCore(
    temp_protection,
    "temp_task",
    5000,
    NULL,
    2,
    &temp_task,
    0);
}

void loop() {
    delay(1000);
}