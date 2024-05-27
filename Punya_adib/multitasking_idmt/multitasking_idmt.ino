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
#define FEMALE_RELAY  4

twai_message_t message;

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

TaskHandle_t current_task, idmt_task;
SemaphoreHandle_t baton;
double realcurrent;
double  Is = 50;

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
      if(message.identifier == 0x18DB33F1){
        long start = millis();
        //printf("\nPGN ID: %x\n", message.identifier);        
        uint8_t currentValue = message.data[1];
        realcurrent = currentValue;
        printf("        Current Value: %.0f A\n", realcurrent);
      }
    }else{
      printf("        Failed to receive message\n");
    }
    Serial.print("        This Task runs on Core : ");
    Serial.println(xPortGetCoreID());
    Serial.print("        Time ");
    Serial.println(millis() - start);  
    vTaskDelay(500/ portTICK_PERIOD_MS);
  }
}

double idmt_veryinverse (float current){
   double k = 13.5;
   double alpha = 1; 
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

    float Ir_mul = realcurrent / Is;

        if (Ir_mul <= 1 ){
          Serial.println("Battery OK");
        }
        else if (Ir_mul > 1 && Ir_mul < 2){
          Serial.println("IDMT Sequence Start");
          double trip_duration = idmt_veryinverse (realcurrent);
          float current_rec = realcurrent;
          int state = 0;
          trip_countdown_start = millis();
          do{
            current_millis = millis();
            if (trip_duration*1000 > (millis() - trip_countdown_start)){
            Serial.println(time_left(trip_duration*1000-(millis()-trip_countdown_start)));
            //delay(100);
            if(realcurrent < current_rec){
              digitalWrite(LED_BUILTIN, LOW);
              break;
            } 
            if(realcurrent > current_rec + 1){
              digitalWrite(LED_BUILTIN, LOW);
              break;
            }               

            if (current_millis - millis_previous_read_LED >= LEDblink_interval){
            millis_previous_read_LED = current_millis;
            if (LEDblink_state == 0) LEDblink_state = 1;  else LEDblink_state = 0;
            digitalWrite(LED_BUILTIN, LEDblink_state);
            }

            }
            else{
              RelayON();
              digitalWrite(FEMALE_RELAY, LOW);
              state = 1;       
            }
            
          }while(state == 0);
        }
        else {
          Serial.println("Instanteous Trip");
          //RelayON();
        }    

    Serial.print("This Task runs on Core : ");
    Serial.println(xPortGetCoreID());
    Serial.print("Time ");
    Serial.println(millis() - start);
    //realcurrent = 0;
    vTaskDelay(1000/ portTICK_PERIOD_MS);
  }
}

// the setup function runs once when you press reset or power the board
void setup() {
  twai_setup_and_install();
  Serial.begin(115200);
  delay(1000);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(FEMALE_RELAY, OUTPUT);

  xTaskCreatePinnedToCore(
    read_current,
    "current_task",
    10000,
    NULL,
    1,
    &current_task,
    0);

  delay(500); // needed to start-up task1

  xTaskCreatePinnedToCore(
    idmt_sequence,
    "idmt_task",
    10000,
    NULL,
    1,
    &idmt_task,
    1);
}

void loop() {
    delay(1000);
}