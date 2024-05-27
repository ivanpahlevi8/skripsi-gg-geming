#include <Arduino.h>
#include "driver/gpio.h"
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/rtc_wdt.h"


// CAN BUS Data SAEJ1939 Based
typedef struct CAN_Inverter_t {

  bool command = false; // Enable/Disable Inverter
  struct {
    bool error = false;               // Display Error is Detected
    bool overVoltage = false;         // Display Over Voltage on Inverter
    bool overCurrent = false;         // Display Over Current on Inverter
    bool overTemp = false;            // Display Over Temperature on Inverter

    bool warning = false;             // Display Warning is Detected
    bool warningOverVoltage = false;  // Display Over Voltage Warning on Inverter
    bool warningCurrent = false;      // Display Over Current Warning on Inverter
    bool warningTemp = false;         // Display Over Temperature Warning on Inverter
  } InverterCondition;
  uint16_t current = 0;       // Current
  uint16_t temp = 0;          // Temp
  uint16_t voltage = 0;       // Voltage
  uint16_t accelerator = 0;
  uint16_t brake = 0;
  uint16_t speed = 0;         // Speed
  uint8_t state = 0;          // InverterCondition
  uint16_t rpm = 0;           // Inverter RPMData
  uint16_t percentTorque = 0; // Percent Torque
  uint16_t torqueDemand = 0; // Torque Demand
  uint16_t actualTorque = 0; // ActualTorque

} CAN_Inverter_t;

CAN_Inverter_t inv_data;
twai_message_t rx_frame;
TaskHandle_t ReadCANHandle = NULL;
TaskHandle_t SendCANHandle = NULL;
const int ledPin =  LED_BUILTIN;// the number of the LED pin

void RTCTimeout(void *arg) {
  while (1) {
    rtc_wdt_feed();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void ReadCAN(void *arg) {
  while (1) {
    //    CANTaskHandler.Update();
    //    Serial.println("test");

    if (twai_receive(&rx_frame, pdMS_TO_TICKS(1000)) == ESP_OK) {
      //      Serial.print("msg :"); Serial.println(rx_frame.identifier);


    }
    delay(1);
  }
}

void SendCAN(void *arg) {
  while (1) {
    twai_message_t inv_msg, inv_msg2, rx_frame, rx_frame2;
    static uint32_t timeout = millis();

    if (millis() - timeout >= 100) {
      inv_msg.identifier = 1023;
      inv_msg.extd = 1;
      inv_msg.data_length_code = 8; // Length
      inv_msg.data[0] = 0x9 ; // Fan Speed times 0.4%
      inv_msg.data[1] = 0x55; // Temp in C times 0.03125
      inv_msg.data[2] = 0xAA; // Temp in C times 0.03125
      inv_msg.data[3] = 0xAA;
      inv_msg.data[4] = 0x00;
      inv_msg.data[5] = 0xAA;
      inv_msg.data[6] = 0;
      inv_msg.data[7] = 0; // Vehicle Speed Limit

      inv_msg2.identifier = 1023;
      inv_msg2.extd = 1;
      inv_msg2.data_length_code = 8; // Length
      inv_msg2.data[0] = 0x0 ; // Fan Speed times 0.4%
      inv_msg2.data[1] = 0x18; // Temp in C times 0.03125
      inv_msg2.data[2] = 0xAA; // Temp in C times 0.03125
      inv_msg2.data[3] = 0x05;
      inv_msg2.data[4] = 0xD2;
      inv_msg2.data[5] = 0x00;
      inv_msg2.data[6] = 0x20;
      inv_msg2.data[7] = 0x33; // Vehicle Speed Limit
      if (twai_transmit(&inv_msg, pdMS_TO_TICKS(1000)) == ESP_OK)  {
      } else {}
      if (twai_transmit(&inv_msg2, pdMS_TO_TICKS(1000)) == ESP_OK)  {
      } else {}
      Serial.println("Sending Data Complete");
      timeout = millis();
    }
    
    if (twai_receive(&rx_frame, pdMS_TO_TICKS(1000)) == ESP_OK) {
      Serial.println("Received First Data");

      inv_data.command = rx_frame.data[0];
      inv_data.voltage = (rx_frame.data[5] << 8) | rx_frame.data[4];
      inv_data.current = (rx_frame.data[7] << 8) | rx_frame.data[6];
      Serial.print("Voltage: "); Serial.println(inv_data.voltage);
    }
    if (twai_receive(&rx_frame, pdMS_TO_TICKS(1000)) == ESP_OK) {
      Serial.println("Received Second Data");
      inv_data.voltage = (rx_frame.data[5] << 8) | rx_frame.data[4];
      inv_data.current = (rx_frame.data[7] << 8) | rx_frame.data[6];
      Serial.print("Voltage: "); Serial.println(inv_data.voltage);
    }

    delay(1);
  }
}


void setup() {
  Serial.begin(115200);
  //    pinMode(2, OUTPUT);
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

  //  xTaskCreate(ReadCAN, "ReadCAN", 2096, NULL, 1, &ReadCANHandle);
  xTaskCreate(SendCAN, "SendCAN", 2096, NULL, 2, &SendCANHandle);

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
