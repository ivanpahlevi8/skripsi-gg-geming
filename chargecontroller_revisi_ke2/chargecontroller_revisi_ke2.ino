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

#define charger_lock 33
#define charger_relay 32
#define vehicle_shift_check 35

#define LED_not_connected 27
#define LED_connecting 26
#define LED_connected 25


// Global calling function for transceiver from battery to charge controller
twai_message_t message_battery_current_voltage;
twai_message_t message_battery_temperature_value; 

// from ev to dc station
twai_message_t id_transmit_102; 
twai_message_t id_transmit_101;
twai_message_t id_transmit_100;

// from dc station to ev 
twai_message_t id_receive_109;
twai_message_t id_receive_108;

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
uint8_t states_byte_5[5];
uint8_t states_byte_4[5];
uint8_t states_value_byte_5 = 0;
uint8_t states_value_byte_4 = 0;


// id 108
uint8_t available_output_voltage; uint8_t available_output_voltage_msb; uint8_t available_output_voltage_lsb;
uint8_t available_output_current_HEX; uint8_t available_output_current;
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


// counting for loop in each core  
long int prev_count_millis = millis();
long int prev_count_millis2 = millis();
long int prev_count_millis3 = millis();
long int prev_count_millis4 = millis();
long int prev_count_millis5 = millis();
long int prev_count_millis6 = millis();
long int prev_count_millis7 = millis();
long int prev_count_millis8 = millis();
long int prev_count_millis9 = millis();
long int prev_count_millis10 = millis();


// Global define 
uint8_t battery_charger_temp;
uint8_t battery_temp_mapping;
uint8_t battery_charger_output_current;
uint8_t battery_charger_output_current_msb;
uint8_t battery_charger_output_current_lsb;
uint8_t battery_charger_output_voltage;
uint8_t battery_charger_output_voltage_msb;
uint8_t battery_charger_output_voltage_lsb;
uint8_t battery_voltage_mapping;
uint8_t battery_current_mapping;
float battery_charger_output_voltage_percent;

uint8_t charging_time;
uint8_t charging_current;
uint8_t battery_capacity;

uint8_t currentsetpoint_condition = 30;
uint8_t currentsetpoint_target = 1;
uint8_t battery_temp_heat_target = 52;      // 52°C   *The optimum ambient temperature for charging a Lithium battery is +5°C to +45°C / 41°F to 113°F
uint8_t battery_low = 60;                   // condition battery in low voltage
uint8_t charger_relay_status;



// Global variables to track condition status
bool receiver_verification_status = false;
bool transmit_verification_status = false;
bool temperature_received = false;
bool voltage_current_received = false;
bool start_exchange = false;
bool identification = false;


// Other variable send to Charger-Off-board
uint8_t Status_for_id_100[8] = {00, 00, 00, 00, 00, 00, 00, 00};
uint8_t Status_for_id_101[8] = {00, 00, 00, 00, 00, 00, 00, 00};
uint8_t Status_for_id_102[8] = {00, 00, 00, 00, 00, 00, 00, 00};
uint8_t Status_for_id_108[8] = {00, 00, 00, 00, 00, 00, 00, 00};
uint8_t Status_for_id_109[8] = {00, 00, 00, 00, 00, 00, 00, 00};

// variable for storing from battery-charging controller-charger off-board
uint8_t CC_Voltage_Current_value[8] = {00, 00, 00, 00, 00, 00, 00, 00};
uint8_t CC_Temperature_value[8] = {00, 00, 00, 00, 00, 00, 00, 00};

uint8_t id_check_DCCCF[8] = {50, 00, 00, 00, 00, 00, 00, 00};
uint8_t id_check_VCCF[8] = {25, 00, 00, 00, 00, 00, 00, 00};


TaskHandle_t battery_current_voltageHandle = NULL;
TaskHandle_t battery_tempHandle = NULL;
TaskHandle_t exchange_data_communication_initialization_receiveHandle = NULL;
TaskHandle_t identification_chargecontroller_chargeroffboardHandle = NULL;
TaskHandle_t exchange_data_communication_initialization_transmitHandle = NULL;


//////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////  BATTERY TO CHARGE CONTROLLER  ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////


void battery_current_voltage(void * argument){
  while(1){
    long int current_count_millis = millis();
    long int current_count_millis2 = millis();

    if(current_count_millis - prev_count_millis >= 1000){
      if(twai_receive(&message_battery_current_voltage, pdMS_TO_TICKS(1000)) == ESP_OK){     
        if(message_battery_current_voltage.identifier == 0x18FD15FE){
          printf("\nID: %X DLC: %d Data Battery Charger 1: \t", message_battery_current_voltage.identifier, message_battery_current_voltage.data_length_code);

          for (int i = 0; i < message_battery_current_voltage.data_length_code; i++) {
              printf("%02X\t", message_battery_current_voltage.data[i]);
          }

          battery_charger_output_current_msb = message_battery_current_voltage.data[4]; 
          battery_charger_output_current_lsb = message_battery_current_voltage.data[3];
          battery_charger_output_current = (battery_charger_output_current_msb << 8) | battery_charger_output_current_lsb;

          battery_charger_output_voltage_msb = message_battery_current_voltage.data[2]; 
          battery_charger_output_voltage_lsb = message_battery_current_voltage.data[1];
          battery_charger_output_voltage = (battery_charger_output_voltage_msb << 8) | battery_charger_output_voltage_lsb;
          
          // After mapping the value from receive
          battery_voltage_mapping = map(battery_charger_output_voltage, 0, 255, 0, 82.15);
          battery_current_mapping = map(battery_charger_output_current, 0, 255, 0, 20);

          battery_charger_output_voltage_percent = (battery_voltage_mapping/82.15)*100;          
          
          printf("\nvoltage:  %d V  | %0.2f % | current:  %d A\n", battery_charger_output_voltage, battery_charger_output_voltage_percent, battery_charger_output_current);
          printf("The mapped value of voltage: %0.2f V | current: %0.2f A\n", battery_voltage_mapping, battery_current_mapping);
        }
      } else{
        printf("\nData battery current-voltage can't open!");
      }
      prev_count_millis = current_count_millis;      
    }

    if(current_count_millis2 - prev_count_millis2 >= 1000){
      if(twai_receive(&message_battery_temperature_value, pdMS_TO_TICKS(1000)) == ESP_OK){        
        if(message_battery_temperature_value.identifier == 0x18FE50FE){
          printf("\nID: %X DLC: %d Data Battery Temperature: \t", message_battery_temperature_value.identifier, message_battery_temperature_value.data_length_code);

          for (int i = 0; i < message_battery_temperature_value.data_length_code; i++) {
              printf("%02X\t", message_battery_temperature_value.data[i]);
          }

          battery_charger_temp = message_battery_temperature_value.data[1] & 0xFF; 
          printf("\nTemperature: %d C\n", battery_charger_temp);
          battery_temp_mapping = map(battery_charger_temp, 0, 255, 0, 40); 
          printf("The mapped value of temperature: %d C\n", battery_temp_mapping);
        } 
      } else{
        printf("\nData battery Temperature can't open!");
      }

    prev_count_millis2 = current_count_millis2;      
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////  CHARGE CONTROLLER TO CHARGER OFF-BOARD  //////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////


void identification_chargecontroller_chargeroffboard(void * argument){
  while(1){
    if(identification != true){
        digitalWrite(LED_connected, LOW);
        digitalWrite(LED_not_connected, LOW);
        digitalWrite(LED_connecting, HIGH);  
        
        long int current_count_millis5 = millis();  

        if(current_count_millis5 - prev_count_millis5 >= 2000){
          printf("\nWake-up message for VCCF!");
          if(twai_receive(&id_receive_109, pdMS_TO_TICKS(10000)) == ESP_OK){      
            if(id_receive_109.identifier == 0x109){
              printf("\nID: %X DLC: %d  Data ID: \t", id_receive_109.identifier, id_receive_109.data_length_code);

              for(int i=0; i < id_receive_109.data_length_code; i++){
                  printf("%d\t", id_receive_109.data[i]);

                if (id_receive_109.data[0] != id_check_DCCCF[0]){
                  receiver_verification_status = false;
                  printf("\tData charger off-board not match!\n");
                  break;
                } else{
                  receiver_verification_status = true;
                } 
              }         
            } else{
              printf("\nFailed to receive message from id 109 identification!\n");
            } 
          }
            
          id_transmit_102.identifier = 0x102;
          id_transmit_102.data_length_code = 8;
            
          for (int k = 0; k < id_transmit_102.data_length_code; k++){
            id_transmit_102.data[k] = id_check_VCCF[k];
          }

          if(twai_transmit(&id_transmit_102, pdMS_TO_TICKS(3000)) == ESP_OK){
            printf("wake-up message for DCCCF!");
            transmit_verification_status = true;
          } else {
            printf("\nFailed to send message id 102 identification!\n");
            transmit_verification_status = false;
          }
          
          if(transmit_verification_status == true && receiver_verification_status == true){
            identification = true;
            digitalWrite(LED_connected, HIGH);
            digitalWrite(LED_not_connected, LOW);
            digitalWrite(LED_connecting, LOW);
          }
        prev_count_millis5 = current_count_millis5;         
        }
      }  
      
      if(transmit_verification_status == false || receiver_verification_status == false){
        identification = false;
      } 
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }    
}
  

void exchange_data_communication_initialization_receive(void * argument){
  while(1){
    if(identification == true){    
      long int current_count_millis6 = millis();   
      long int current_count_millis7 = millis();

      if(current_count_millis6 - prev_count_millis6 >= 1000){
        if(twai_receive(&id_receive_108, pdMS_TO_TICKS(1000)) == ESP_OK){
          if(id_receive_108.identifier == 0x108){
            printf("\nID: %X DLC: %d Data: \t", id_receive_108.identifier, id_receive_108.data_length_code);

            for(int i=0; i < id_receive_108.data_length_code; i++){
              printf("%02X\t", id_receive_108.data[i]); 
            }
  
            available_output_current_HEX = id_receive_108.data[3];
            available_output_voltage_msb = id_receive_108.data[2]; 
            available_output_voltage_lsb = id_receive_108.data[1];
            available_output_current = available_output_current_HEX & 0xFF;
            available_output_voltage = (available_output_voltage_msb << 8) | available_output_voltage_lsb;
            charging_power = available_output_voltage * available_output_current;
                          
            printf("\nAvailable output voltage DC Station:  %0.2f V\tAvailable output current DC Station:  %0.2f A\n", available_output_voltage, available_output_current);
          }
        } else{
          printf("\nData 108 not available!");
        }
        prev_count_millis6 = current_count_millis6;           
      }

      if(current_count_millis7 - prev_count_millis7 >= 1000){
        if(twai_receive(&id_receive_109, pdMS_TO_TICKS(1000)) == ESP_OK){
          if(id_receive_109.identifier == 0x109){
            printf("\nID: %X DLC: %d Data: \t", id_receive_109.identifier, id_receive_109.data_length_code);

            for(int i=0; i < id_receive_109.data_length_code; i++){
              printf("%02X\t", id_receive_109.data[i]); 
            }

            case_109_5 = id_receive_109.data[5] & 0xFF;
            switch (case_109_5){
              case 0:
                printf("\nStation status: STANDBY  Station malfunction: NORMAL Vehicle connector lock: UNCLOCKED  Battery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");                        
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 0;
                break;
              case 1:
                printf("\nStation status: CHARGING  Station malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 0;               
                break;
              case 2:
                printf("\nStation status: STANDBY  Station malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");               
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 0;               
                break;
              case 3:
                printf("\nStation status: CHARGING  Station malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");               
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 0;                               
                break;
              case 4:
                printf("\nStation status: STANDBY  Station malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");               
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 0;               
                break;
              case 5:
                printf("\nStation status: CHARGING  Station malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");               
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 0;               
                break;
              case 6:
                printf("\nStation status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");               
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 0;               
                break;
              case 7:
                printf("\nStation status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");               
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 0;               
                break;
              case 8:
                printf("\nStation status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");                
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 0;               
                break;
              case 9:
                printf("\nStation status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n"); 
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 0;              
                break;
              case 10:
                printf("\nStation status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n"); 
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 0;              
                break;
              case 11:
                printf("\nStation status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 0;               
                break;
              case 12:
                printf("\nStation status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");    
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 0;           
                break;
              case 13:
                printf("\nStation status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");   
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 0;            
                break;
              case 14:
                printf("\nStation status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 0;               
                break;
              case 15:
                printf("\nStation status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 0;               
                break;
              case 16:
                printf("\nStation status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n"); 
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 0;              
                break;
              case 17:
                printf("\nStation status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n"); 
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 0;              
                break;
              case 18:
                printf("\nStation status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n"); 
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 0;              
                break;
              case 19:
                printf("\nStation status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n");   
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 0;            
                break;
              case 20:
                printf("\nStation status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n");   
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 0;            
                break;
              case 21:
                printf("\nStation status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n");    
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 0;           
                break;
              case 22:
                printf("\nStation status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n");  
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 0;                  
                break;
              case 23:
                printf("\nStation status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n");
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 0;                           
                break;
              case 24:
                printf("\nStation status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n");
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 0;               
                break;
              case 25:
                printf("\nStation status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n");
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 0;                 
                break;
              case 26:
                printf("\nStation status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n"); 
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 0;                
                break;
              case 27:
                printf("\nStation status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n"); 
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 0;                
                break;
              case 28:
                printf("\nStation status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n");  
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 0;               
                break;
              case 29:
                printf("\nStation status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n"); 
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 0;                 
                break;
              case 30:
                printf("\nStation status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n");
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 0;                  
                break;
              case 31:
                printf("\nStation status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n");  
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 0;                
                break;
              case 32:
                printf("\nStation status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n");    
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 1;           
                break;
              case 33:
                printf("\nStation status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n");
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 1;                  
                break;
              case 34:
                printf("\nStation status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n");   
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 1;               
                break;
              case 35:
                printf("\nStation status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n");   
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 1;               
                break;
              case 36:
                printf("\nStation status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n"); 
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 1;                 
                break;
              case 37:
                printf("\nStation status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n"); 
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 1;                
                break;
              case 38:
                printf("\nStation status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n");     
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 1;             
                break;
              case 39:
                printf("\nStation status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n"); 
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 1;                 
                break;
              case 40:
                printf("\nStation status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n");  
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 1;                
                break;
              case 41:
                printf("\nStation status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n"); 
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 1;               
                break;
              case 42:
                printf("\nStation status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n");   
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 1;             
                break;
              case 43:
                printf("\nStation status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n");  
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 1;              
                break;
              case 44:
                printf("\nStation status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n");  
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 1;              
                break;
              case 45:
                printf("\nStation status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n");  
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 1;              
                break;
              case 46:
                printf("\nStation status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n");     
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 1;           
                break;
              case 47:
                printf("\nStation status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n");   
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 1;             
                break;
              case 48:
                printf("\nStation status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n");        
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 1;        
                break;
              case 49:
                printf("\nStation status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n");        
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 1;              
                break;
              case 50:
                printf("\nStation status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n"); 
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 1;                     
                break;
              case 51:
                printf("\nStation status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n");        
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 1;              
                break;
              case 52:
                printf("\nStation status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n"); 
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 1;                    
                break;
              case 53:
                printf("\nStation status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n"); 
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 1;                     
                break;
              case 54:
                printf("\nStation status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n"); 
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 1;                     
                break;
              case 55:
                printf("\nStation status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n");      
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 1;                
                break;
              case 56:
                printf("\nStation status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n");  
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 1;                    
                break;
              case 57:
                printf("\nStation status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n");               
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 1;    
                break;
              case 58:
                printf("\nStation status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n");     
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 1;              
                break;
              case 59:
                printf("\nStation status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n");  
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 1;                 
                break;
              case 60:
                printf("\nStation status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n"); 
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 1;                  
                break;
              case 61:
                printf("\nStation status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n");  
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 1;             
                break;
              case 62:
                printf("\nStation status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n");
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 1;               
                break;
              case 63:
                printf("\nStation status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n");
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 1;               
                break;
              default:
                printf("\nInvalid bit data 109 byte 5!\n");
                break;
              }


          // for condition relay all closed
            if(station_malfunction == 0 && vehicle_connector_lock == 1 && battery_incompatibility == 0 && charging_system_malfunction == 0 && charger_stop_control == 0){
              digitalWrite(charger_lock, LOW);
              digitalWrite(charger_relay, LOW);
              charger_relay_status = 1; 

            } else if(station_malfunction == 1 || vehicle_connector_lock == 0 || battery_incompatibility == 1 || charging_system_malfunction == 1 || charger_stop_control == 1){
              digitalWrite(charger_relay, HIGH);
              charger_relay_status = 0;
              digitalWrite(charger_lock, HIGH);
            }
          }
        } else{
        printf("\nData 109 not available\n");
        }
        prev_count_millis7 = current_count_millis7;           
      }
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}


void exchange_data_communication_initialization_transmit(void * argument){
  while(1){
    if(identification == true){
      operation_id_100();
      operation_id_101();
      operation_id_102();

      long int current_count_millis8 = millis();
      long int current_count_millis9 = millis();
      long int current_count_millis10 = millis();

      if(current_count_millis8 - prev_count_millis8 >= 1000){
        message_transfer(&id_transmit_100, 0x100, 8, Status_for_id_100);
        prev_count_millis8 = current_count_millis8;         
      }
      if(current_count_millis9 - prev_count_millis9 >= 1000){
        message_transfer(&id_transmit_101, 0x101, 8, Status_for_id_101);
        prev_count_millis9 = current_count_millis9;         
      }
      if(current_count_millis10 - prev_count_millis10 >= 1000){
        message_transfer(&id_transmit_102, 0x102, 8, Status_for_id_102);
        prev_count_millis10 = current_count_millis10;         
      }      
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////  OTHER  //////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////



void operation_id_100(){  
  // constant_of_charging_rate_indication;

  // Maximum charge battery before full charge from max voltage battery (for simulation just 82V from battery GESITS)
  maximum_battery_voltage = 82.15;                                                                                    // voltage set point 
  maximum_battery_voltage_msb = (maximum_battery_voltage >> 8) & 0xFF;
  maximum_battery_voltage_lsb = (maximum_battery_voltage & 0xFF);

  Status_for_id_100[5] = maximum_battery_voltage_msb;
  Status_for_id_100[4] = maximum_battery_voltage_lsb;
}

void operation_id_101(){
  // rated capacity value mAh
  rated_capacity_of_battery = 1.3968;                                                     //V x Ah = 82 x 20Ah * 0,1(resolution 0.1 kWh/bit)
  uint8_t rated_capacity_of_battery_after = rated_capacity_of_battery * 0.1;
  rated_capacity_of_battery_msb = (rated_capacity_of_battery_after >> 8) & 0xFF;
  rated_capacity_of_battery_lsb = (rated_capacity_of_battery_after & 0xFF);
  
  Status_for_id_101[6] = rated_capacity_of_battery_msb; 
  Status_for_id_101[5] = rated_capacity_of_battery_lsb;
  
  // maximum_charging_time = EV battery capacity (kWh) / Charging power
  maximum_charging_time = rated_capacity_of_battery / charging_power;  //charging_power = 10 

  int minutes = maximum_charging_time * 60;                                               // Convert hours to minutes
  int seconds = ((maximum_charging_time * 3600) - (minutes * 60)) * 10;                   // Convert remaining to seconds & Multiply by 10 to get the actual seconds  
  // int seconds = 10;                                                                    // Convert remaining to seconds & Multiply by 10 to get the actual seconds  

  minutes = min(minutes, 255);
  seconds = min(seconds / 10, 254);                                                       // divide by 10 because each bit represents 10 seconds
  
  Status_for_id_101[2] = minutes & 0xFF;
  Status_for_id_101[1] = seconds & 0xFF;

  //Print the values to the serial monitor for debugging
  printf("\nMaximum Charging Time \tMinutes: %d, Seconds: %d\n", Status_for_id_101[2],Status_for_id_101[1]);
}


void operation_id_102(){

  target_battery_voltage = 81.11;                                       // volt
  target_battery_voltage_msb = (target_battery_voltage >> 8) & 0xFF;
  target_battery_voltage_lsb = (target_battery_voltage & 0xFF);


  // for battery temp condition
  if (battery_temp_mapping < battery_temp_heat_target){
    high_battery_temperature = 0;
  } else if (battery_temp_mapping >= battery_temp_heat_target){
    high_battery_temperature = 1;
  }

  // for charging current request condition
  if (vehicle_charging_enable == 1 && charger_lock == LOW){
    charging_current_request = 20;                                      //20 A  fast charge
  } else if (vehicle_charging_enable == 0 && charger_lock == HIGH){
    charging_current_request = 0;                                       //0 A   condition fault
  } else if (high_battery_temperature == 1){
    charging_current_request = 10;                                      //10 A  condition heat temp
  } else if (battery_voltage_mapping == target_battery_voltage){
    charging_current_request = 0;                                       //0 A   end of charge to CV
  }


  // ************ condition 102[5]  ************
  // for charging enable condition
  if (charger_stop_control == 0 || battery_incompatibility == 0 || station_malfunction == 0 || charging_system_malfunction == 0){
    vehicle_charging_enable == 1;   //enable
  } else if(battery_charger_output_voltage == maximum_battery_voltage || charger_stop_control == 1 || battery_incompatibility == 1 || charging_system_malfunction == 1 || station_malfunction == 1){
    vehicle_charging_enable == 0;   //disable
  }

  // for vehicle shift level position     *use button just for try
  if(digitalRead(vehicle_shift_check) == HIGH){
    vehicle_shift_lever_position = 1;
  } else if(digitalRead(vehicle_shift_check) == LOW){
    vehicle_shift_lever_position = 0;
  }

  // for charging system fault  *condition when the current not detect or there's no charging 

  
  // for vehicle status
  if (charger_relay_status == 1){
    vehicle_status = 0;
  } else if (charger_relay_status == 0){
    vehicle_status = 1;
  }

  // for normal stop request before charging



  // *******************************************



  // ************ condition 102[4]  ************
  // for battery overvoltage
  if (battery_voltage_mapping > maximum_battery_voltage){
    battery_overvoltage = 1;
  } else if (battery_voltage_mapping <= maximum_battery_voltage){
    battery_overvoltage = 0;  
  }

  // for battery undervoltage
  if (battery_voltage_mapping < battery_low){
    battery_undervoltage = 1;
  } else if (battery_voltage_mapping >= battery_low){
    battery_undervoltage = 0;  
  }

  // for battery current deviation error        *must compared beetween current requested and actual current send to battery (from sensor)

  // for battery voltage deviation error        *must compared beetween voltage station and actual voltage send to battery (from sensor)

  // *******************************************


  states_byte_4[0] = vehicle_charging_enable;
  states_byte_4[1] = vehicle_shift_lever_position;
  states_byte_4[2] = charging_system_fault;
  states_byte_4[3] = vehicle_status;
  states_byte_4[4] = normal_stop_request_before_charging;  

  for (int m = 0; m < 5; m++) {
    states_value_byte_4 |= (states_byte_4[m] << m);
  }

  states_byte_5[0] = battery_overvoltage;
  states_byte_5[1] = battery_undervoltage;
  states_byte_5[2] = battery_current_deviation_error;
  states_byte_5[3] = high_battery_temperature;
  states_byte_5[4] = battery_voltage_deviation_error;

  for (int l = 0; l < 5; l++) {
    states_value_byte_5 |= (states_byte_5[l] << l);
  }

  Status_for_id_102[5] = states_value_byte_4;
  Status_for_id_102[4] = states_value_byte_5;
  Status_for_id_102[3] = charging_current_request;
  Status_for_id_102[2] = target_battery_voltage_msb; 
  Status_for_id_102[1] = target_battery_voltage_lsb;
  Status_for_id_102[0] = 25;                                                          // protocol number
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// TWAI TRANSCEIVER  //////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////


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
    printf("Message queued for transmission");
  } else {
    printf("Failed to send message");
  }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////  SET UP  //////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////


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

  pinMode(charger_lock, OUTPUT);
  pinMode(charger_relay, OUTPUT);
  pinMode(vehicle_shift_check, OUTPUT);
  pinMode(LED_not_connected, OUTPUT); digitalWrite(LED_not_connected, HIGH);
  pinMode(LED_connecting, OUTPUT); digitalWrite(LED_connecting, LOW);
  pinMode(LED_connected, OUTPUT); digitalWrite(LED_connected, LOW);

  xTaskCreate(battery_current_voltage, "battery_current_voltage", 7000, NULL, 1, &battery_current_voltageHandle);
  xTaskCreate(identification_chargecontroller_chargeroffboard, "identification_chargecontroller_chargeroffboard", 2096, NULL, 2, &identification_chargecontroller_chargeroffboardHandle);
  xTaskCreate(exchange_data_communication_initialization_receive, "exchange_data_communication_initialization_receive", 2096, NULL, 3, &exchange_data_communication_initialization_receiveHandle);
  xTaskCreate(exchange_data_communication_initialization_transmit, "exchange_data_communication_initialization_transmit", 2096, NULL, 4, &exchange_data_communication_initialization_transmitHandle);
}

void loop(){

}