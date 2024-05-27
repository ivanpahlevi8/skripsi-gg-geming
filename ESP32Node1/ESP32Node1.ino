#include <Arduino.h>
#include "ESPVCU.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/rtc_wdt.h"


#define GPIO_NUM1 12
#define GPIO_NUM2 14
#define GPIO_NUM3 26
#define GPIO_NUM4 13
#define GPIO_NUM5 27
#define GPIO_NUM6 34
#define GPIO_NUM7 25
#define GPIO_NUM8 32
#define GPIO_NUM9 39
#define GPIO_NUM10 25

#define PERCENTAGEVOLTAGE 40    // Percentage to 24 V, Use 50 for 12/24 with offset -5


twai_message_t rx_frame;
CANCompiler CANTaskHandler;
int RPM_data, Speed_data, Temp_data = 0;
bool digitalPin2, digitalPin4, digitalPin8, digitalPin9, digitalPin10 = false;
int digitalPin2Prev, digitalPin4Prev, digitalPin8Prev = false;
float velocity = 0;
uint8_t accelerator, brake = 0;
int btn2HitCount, btn4HitCount, btn8HitCount = 0;
void sendKeyData();
void sendAndroidData();
TaskHandle_t ReadCANHandle = NULL;
TaskHandle_t SendCANHandle = NULL;
TaskHandle_t GPIOCANHandle = NULL;
TaskHandle_t ERRORCANHandle = NULL;
TaskHandle_t RTCA = NULL;

const int ledPin =  LED_BUILTIN;// the number of the LED pin

void RTCTimeout(void *arg) {
  while (1) {
    rtc_wdt_feed();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void GPIORead(void *arg) {
  //  rtc_wdt_protect_off();    // Turns off the automatic wdt service
  //  rtc_wdt_disable();
  while (1) {
    int analogPin1 = map(analogRead(GPIO_NUM1), 0, 4096, 0, 100);
    int analogPin2 = map(analogRead(GPIO_NUM2), 0, 4096, 0, 100);
    int analogPin3 = map(analogRead(GPIO_NUM3), 0, 4096, 0, 100);
    int analogPin4 = map(analogRead(GPIO_NUM4), 0, 4096, 0, 100);
    int analogPin6 = map(analogRead(GPIO_NUM6), 0, 4096, 0, 100);
    int analogPin7 = map(analogRead(GPIO_NUM7), 0, 4096, 0, 100);
    int analogPin8 = map(analogRead(GPIO_NUM8), 0, 4096, 0, 100);
    int analogPin9 = map(analogRead(GPIO_NUM9), 0, 4096, 0, 100);
    int analogPin10 = map(analogRead(GPIO_NUM10), 0, 4096, 0, 100);
    digitalPin2Prev = digitalPin2; digitalPin4Prev = digitalPin4; digitalPin8Prev = digitalPin8;
    // Read Digital Data States
    if (analogPin2 > PERCENTAGEVOLTAGE) digitalPin2 = true;
    else digitalPin2 = false;
    if (analogPin4 > PERCENTAGEVOLTAGE) digitalPin4 = true;
    else digitalPin4 = false;
    if (analogPin8 > PERCENTAGEVOLTAGE) digitalPin8 = true;
    else digitalPin8 = false;
    if (analogPin9 > PERCENTAGEVOLTAGE) digitalPin9 = true;
    else digitalPin9 = false;
    if (analogPin10 > PERCENTAGEVOLTAGE) digitalPin10 = true;
    else digitalPin10 = false;

    // Convert Analog to Digital Data
    if (digitalPin2Prev == HIGH && digitalPin2 == LOW) {
      btn2HitCount += 1;
    }
    if (digitalPin4Prev == HIGH && digitalPin4 == LOW ) {
      btn4HitCount += 1;
    }
    if (digitalPin8Prev == HIGH && digitalPin8 == LOW ) {
      btn8HitCount += 1;
    }

    if (digitalPin10 && digitalPin9) CANTaskHandler.keyData.keyPos = 3;
    else if (digitalPin9 || digitalPin10) CANTaskHandler.keyData.keyPos = 2;
    else CANTaskHandler.keyData.keyPos = 1;
    // Read Inverter Data

    if (btn2HitCount % 2 == 0) {
      accelerator = map(analogPin3, 0, PERCENTAGEVOLTAGE + 10, 0, 100);

      CANTaskHandler.inverterData.accelerator = accelerator;
    }
    else if (btn2HitCount % 2 == 1) {
      brake = map(analogPin3, 0, PERCENTAGEVOLTAGE + 10, 0, 100);

      CANTaskHandler.inverterData.brake = brake;

    }

    if (btn4HitCount % 2 == 0) {
      CANTaskHandler.inverterData.voltage = map(analogPin1, 0, PERCENTAGEVOLTAGE + 10, 0, 1400);

      CANTaskHandler.inverterData.accelerator = accelerator;
    }
    else if (btn4HitCount % 2 == 1) {
      CANTaskHandler.inverterData.current = map(analogPin1, 0, PERCENTAGEVOLTAGE + 10, 0, 280);


    }

    CANTaskHandler.inverterData.temp = map(analogPin6, 0, PERCENTAGEVOLTAGE + 10, 0, 200);

    if (btn2HitCount == 2) btn2HitCount = 0;
    if (btn4HitCount == 2) btn4HitCount = 0;
    if (btn8HitCount == 2) btn8HitCount = 0;

    if (CANTaskHandler.inverterData.command) {
      CANTaskHandler.inverterData.accelerator = 0;
      CANTaskHandler.inverterData.brake = 0;
      CANTaskHandler.inverterData.speed = 0;
      velocity = 0;
    }

    // Velocity Counting
    velocity = velocity + (accelerator - brake) * 0.01;
    if (velocity > 355) velocity = 355;
    if (velocity < 0 ) velocity = 0;
    uint16_t velo_to_uint16_t = (uint16_t) (velocity * 100);
    CANTaskHandler.inverterData.speed = velo_to_uint16_t;
    CANTaskHandler.inverterData.rpm  = velocity / (0.001885 * 25);
    static uint32_t timeout = millis();
    if (millis() - timeout >= 50) {
      //      Serial.print("Inverter Warning: "); Serial.println(CANTaskHandler.inverterData.InverterCondition.warning);
      //      Serial.print("Inverter Error: "); Serial.println(CANTaskHandler.inverterData.InverterCondition.error);
      //      Serial.print("Inverter Current: "); Serial.println(CANTaskHandler.inverterData.current);

      //            Serial.print("Digital 1: "); Serial.print(digitalPin2);
      //            Serial.print(" GPIO 1: "); Serial.print(analogPin1);
      //            Serial.print(" GPIO 3: "); Serial.print(analogPin3);
      //            Serial.print("Digital 2: "); Serial.println(digitalPin4);
      //            Serial.print(" GPIO 6: "); Serial.print(analogPin6);
      //            Serial.print("Digital 3: "); Serial.print(digitalPin8);
      //            Serial.print(" Digital 4: "); Serial.print(digitalPin9);
      //            Serial.print(" Digital 5: "); Serial.println(digitalPin10);
      //            Serial.print("GPIO 1: "); Serial.print(analogPin1);
      //            Serial.print(" GPIO 2: "); Serial.print(analogPin2);
      //            Serial.print(" GPIO 3: "); Serial.print(analogPin3);
      //            Serial.print(" GPIO 4: "); Serial.println(analogPin4);
      //      //      Serial.print("GPIO 5: "); Serial.print(analogPin5);
      //            Serial.print(" GPIO 6: "); Serial.print(analogPin6);
      //            Serial.print(" GPIO 7: "); Serial.print(analogPin7);
      //            Serial.print(" GPIO 8: "); Serial.println(analogPin8);
      //            Serial.print("GPIO 9: "); Serial.print(analogPin9);
      //            Serial.print(" GPIO 10: "); Serial.print(analogPin10);

      //      Serial.print("Velocity: "); Serial.println(CANTaskHandler.inverterData.speed);
      //      Serial.print("RPM: "); Serial.println(CANTaskHandler.inverterData.rpm);
            Serial.print("Temp: ");
            Serial.println(CANTaskHandler.inverterData.temp);
      //    #ifdef INVERTER
      //        if (!(CANTaskHandler.inverterData.timestampCheckMillis.timeout)) {
      //          Serial.print("Inverter Data Timestamp: "); Serial.println(CANTaskHandler.inverterData.timestampCheckMillis.timestampDiff);
      //        }
      //        else {
      //          Serial.println("Inverter Data Timeout");
      //        }
      //        if (!(CANTaskHandler.inverterData.timestamp2CheckMillis.timeout)) {
      //          Serial.print("Inverter 2Timestamp: "); Serial.println(CANTaskHandler.inverterData.timestamp2CheckMillis.timestampDiff);
      //        }
      //        else {
      //          Serial.println("Inverter 2 Data Timeout");
      //        }
      //    #endif
      timeout = millis();
    }

    delay(10);
  }
}

void ErrorCAN(void *arg) {
  while (1) {
    CANTaskHandler.TimeOut();
    delay(20);
  }
}
void ReadCAN(void *arg) {
  while (1) {
    //    CANTaskHandler.Update();
    //    Serial.println("test");

    if (twai_receive(&rx_frame, pdMS_TO_TICKS(1000)) == ESP_OK) {
      //      Serial.print("msg :"); Serial.println(rx_frame.identifier);

      CANTaskHandler.ReceiveData(rx_frame);

    }
    delay(1);
  }
}

void SendCAN(void *arg) {
  while (1) {
    twai_message_t keyMsg, inverterMsg, inverterMsg2;
    static uint32_t timeout = millis();
    static uint32_t timeout2 = millis();

    if (millis() - timeout >= 50) {
      CANTaskHandler.InverterSend(&inverterMsg);
      CANTaskHandler.Inverter2Send(&inverterMsg2);
      if (twai_transmit(&inverterMsg, pdMS_TO_TICKS(1000)) == ESP_OK)  {
      } else {}
      if (twai_transmit(&inverterMsg2, pdMS_TO_TICKS(1000)) == ESP_OK)  {
      } else {}

      timeout = millis();
    }
    if (millis() - timeout2 >= 200) {
      CANTaskHandler.KeySend(&keyMsg);
      if (twai_transmit(&keyMsg, pdMS_TO_TICKS(1000)) == ESP_OK)  {
      } else {}
      timeout2 = millis();
    }
    delay(1);
  }
}


void setup() {
  Serial.begin(115200);
  //    pinMode(2, OUTPUT);
  pinMode(GPIO_NUM1, INPUT);
  pinMode(GPIO_NUM2, INPUT);
  pinMode(GPIO_NUM3, INPUT);
  pinMode(GPIO_NUM4, INPUT);
  pinMode(GPIO_NUM5, INPUT);
  pinMode(GPIO_NUM6, INPUT);
  pinMode(GPIO_NUM7, INPUT);
  pinMode(GPIO_NUM8, INPUT);
  pinMode(GPIO_NUM9, INPUT);
  pinMode(GPIO_NUM10, INPUT);

  pinMode(ledPin, OUTPUT);
  //  rtc_wdt_enable();         // Turn it on manually


  //  rtc_wdt_enable();         // Turn it on manually
  //
  rtc_wdt_set_time(RTC_WDT_STAGE0, 2000);  // Define how long you desire to let dog wait.
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_4, GPIO_NUM_15, TWAI_MODE_NORMAL);
//  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();

  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  //Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("Driver installed");
  } else {
    Serial.println("Failed to install driver");
  }

  //Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.print("Driver started\n");
  } else {
    Serial.print("Failed to start driver\n");
  }

  xTaskCreate(ReadCAN, "ReadCAN", 2096, NULL, 1, &ReadCANHandle);
  xTaskCreate(SendCAN, "SendCAN", 2096, NULL, 2, &SendCANHandle);
  xTaskCreate(GPIORead, "GPIOCAN", 2096, NULL, 3, &GPIOCANHandle);
  xTaskCreate(ErrorCAN, "ERRORCAN", 2096, NULL, 4, &ERRORCANHandle);
  //   xTaskCreate(RTCTimeout, "RTC", 2096, NULL, 4, &RTCA);

}

void loop() {
  //        Serial.print("ID: "); Serial.println(rx_frame.identifier,HEX);
  //         for(int i = 0; i < 8; i++){
  //             Serial.print(rx_frame.data[i], HEX); Serial.print(" ");
  //
  //        }



}

//void sendAndroidData(){
//  Serial.print("RPM="); Serial.print(RPM_data); Serial.print(",");
//  Serial.print("Speed="); Serial.print(Speed_data); Serial.print(",");
//  Serial.print("Temp="); Serial.print(Temp_data); Serial.println(",");
//
//  }
