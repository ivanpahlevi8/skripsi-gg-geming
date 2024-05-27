/*Exchanged information for d.c. charging control
  a-1   ->  Current request for the controlled current charging (CCC) system
  a-2   ->  Voltage request for the controlled voltage charging (CVC) system
  a-3   ->  Maximum rated voltage of d.c. EV charging station
  a-4   ->  Maximum rated current of d.c. EV charging station
  b-1   ->  Communication protocol
  b-2   ->  Maximum voltage limit of EV
  b-3   ->  EV minimum current limit, only for the controlled voltage charging (CVC) system
  c     ->  Insulation test result
  d     ->  Short circuit test before charging                                                  
  e     ->  Charging stopped by user
  f     ->  EVSE real time available load current (optional)
  g     ->  Loss of digital communication
  h-1   ->  Zero current confirmed
  h-2   ->  Welding detection                                                                   (Not used)
*/


/*
  ID 0x100      > {00 00 00 00 00 00 00 00} 
                  byte 4 & 5  : maximum battery voltage
                  byte 6      : constan of charging rate indication (indikasi nilai tetap charging rate sebagai maksimum charging rate 100%)

  ID 0x101      > {00 00 00 00 00 00 00 00}
                  byte 1      : maximum charging time (set by 10s)          
                  byte 2      : maximum charging time (set by minute)       
                  byte 3      : estimated charging time
                  byte 5 & 6  : rated capacity of battery 

  ID 0x102      > {00 00 00 00 00 00 00 00}  
                  byte 0      : control protocol number (EV Correspondens)
                  byte 1 & 2  : target battery voltage (targeted charging voltage at the vehicle inlet terminal)
                  byte 3      : charging current-request
                  byte 4      : 
                        (0)   : battery overvoltage (with status flag)
                        (1)   : battery undervolatge (with status flag)
                        (2)   : battery current deviation error
                        (3)   : high battery temperature
                        (4)   : battery voltage deviation error
                  byte 5      : 
                        (0)   : vehicle charging enabled
                        (1)   : vehicle shift lever position                                      
                        (2)   : charging system fault
                        (3)   : vehicle status                                                    
                        (4)   : normal stop request before charging
                  byte 6      : charging rate (charging rate of vehicle)   

  ID 0x108      > {00 00 00 00 00 00 00 00}
                  byte 0      : EV contactor welding detection support identifier                 
                  byte 1 & 2  : available output voltage
                  byte 3      : available output current
                  byte 4 & 5  : threshold voltage

  ID 0x109      > {00 00 00 00 00 00 00 00}
                  byte 0      : control protocol number (charging sequences that the station deals with)
                  byte 1 & 2  : output voltage
                  byte 3      : output current
                  byte 5      :
                        (0)   : station status
                        (1)   : station malfunction 
                        (2)   : vehicle connector lock
                        (3)   : battery incompability
                        (4)   : charging system malfunction
                        (5)   : charger stop control
                  byte 6      : remaining charging time (counted by 10s)         (oke)
                  byte 7      : remaining charging time (counted by min)         (oke)
*/


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

#define charger_lock 15
#define charger_relay 4

#define LED_not_connected 27
#define LED_connecting 26
#define LED_connected 25

#define switch_vehicle_shifter 4



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


// counting for loop in each core  
long int prev_count_millis = millis();
long int prev_count_millis2 = millis();
long int prev_count_millis3 = millis();
long int prev_count_millis4 = millis();
long int prev_count_millis5 = millis();
long int prev_count_millis6 = millis();
long int prev_count_millis7 = millis();
long int prev_count_millis8 = millis();


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
uint8_t battery_temp_heat_target = 45;      // 45°C   *The optimum ambient temperature for charging a Lithium battery is +5°C to +45°C / 41°F to 113°F
uint8_t battery_temp_overheat = 50;
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

uint8_t id_check_DCCCF[8] = {50, 00, 00, 00, 00, 00, 00, 00};
uint8_t id_check_VCCF[8] = {25, 00, 00, 00, 00, 00, 00, 00};


// variable for storing from battery-charging controller-charger off-board
uint8_t CC_Voltage_Current_value[8] = {00, 00, 00, 00, 00, 00, 00, 00};
uint8_t CC_Temperature_value[8] = {00, 00, 00, 00, 00, 00, 00, 00};




//////////////////////////////////////////////  TWAI INSTALL AND SETUP  //////////////////////////////////////////////


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


////////////////////////////////////////////  BATTERY TO CHARGE CONTROLLER  //////////////////////////////////////////////


void battery_current_voltage_temp(){
    long int current_count_millis = millis();

    if(current_count_millis - prev_count_millis >= 1300){
      if(twai_receive(&message_battery_current_voltage, pdMS_TO_TICKS(1300)) == ESP_OK){        
        if(message_battery_current_voltage.identifier == 0x18FD15FE){
          printf("\nID: %X DLC: %d \nData Battery Charger 1: \t", message_battery_current_voltage.identifier, message_battery_current_voltage.data_length_code);

          for (int i = 0; i < message_battery_current_voltage.data_length_code; i++) {
              printf("%02X\t", message_battery_current_voltage.data[i]);
          }

          battery_charger_output_current_msb = message_battery_current_voltage.data[4]; 
          battery_charger_output_current_lsb = message_battery_current_voltage.data[3];
          battery_charger_output_current = (battery_charger_output_current_msb << 8) | battery_charger_output_current_lsb;

          battery_charger_output_voltage_msb = message_battery_current_voltage.data[2]; 
          battery_charger_output_voltage_lsb = message_battery_current_voltage.data[1];
          battery_charger_output_voltage = (battery_charger_output_voltage_msb << 8) | battery_charger_output_voltage_lsb;
          
          battery_charger_output_voltage_percent = ((float)battery_charger_output_voltage / 0xFFFF) * 100;
          
          printf("\nvoltage:  %d V  | %0.2f '%'\ncurrent:  %d A\n", battery_charger_output_voltage, battery_charger_output_voltage_percent, battery_charger_output_current);

          // After mapping the value from receive
          battery_voltage_mapping = map(battery_charger_output_voltage, 0, 255, 0, 80);
          battery_current_mapping = map(battery_charger_output_current, 0, 255, 0, 50);
          
          printf("The mapped value of voltage: %d V | current: %d A\n", battery_voltage_mapping, battery_current_mapping);
        }
      } else{
        printf("\nData battery current-voltage can't open!");
      }

      if(twai_receive(&message_battery_temperature_value, pdMS_TO_TICKS(1400)) == ESP_OK){        
        if(message_battery_temperature_value.identifier == 0x18FE50FE){

          printf("\nID: %X DLC: %d \nData Battery Temperature: \t", message_battery_temperature_value.identifier, message_battery_temperature_value.data_length_code);

          for (int i = 0; i < message_battery_temperature_value.data_length_code; i++) {
              printf("%02X\t", message_battery_temperature_value.data[i]);
          }

          battery_charger_temp = message_battery_temperature_value.data[1]; 
          printf("\nTemperature: %d C\n", battery_charger_temp); 

          // After mapping the value from receive
          battery_temp_mapping = map(battery_charger_temp, 0, 255, 0, 40); 
          printf("The mapped value of temperature: %d C\n", battery_temp_mapping);

        }
      } else{
        printf("\nData battery Temperature can't open!");
      }

      prev_count_millis = current_count_millis;      
    }
}




//////////////////////////////////////////////  CHARGE CONTROLLER TO CHARGER OFF-BOARD  //////////////////////////////////////////////


void identification_chargecontroller_chargeroffboard(){
  
  while(identification == false){
    delay(1000);            ...........................
    digitalWrite(LED_connected, LOW);
    digitalWrite(LED_not_connected, LOW);
    digitalWrite(LED_connecting, HIGH);  
    
      long int current_count_millis5 = millis();
      printf("\nWake-up message for VCCF!\n");  

      if(current_count_millis5 - prev_count_millis5 >= 1500){
        if(twai_receive(&id_receive_109, pdMS_TO_TICKS(1500)) == ESP_OK){      
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
            printf("Failed to receive message from id 109 identification!\n");
          } 
        }
        
        id_transmit_102.identifier = 0x102;
        id_transmit_102.data_length_code = 8;
        
        for (int k = 0; k < id_transmit_102.data_length_code; k++){
          id_transmit_102.data[k] = id_check_VCCF[k];
        }

        if(twai_transmit(&id_transmit_102, pdMS_TO_TICKS(1600)) == ESP_OK){
          printf("\nwake-up message for DCCCF!\n");
          transmit_verification_status = true;
        } else {
          printf("\tFailed to send message id 102 identification!\n");
          transmit_verification_status = false;
        }
      
        if(transmit_verification_status == true || receiver_verification_status == true){
          identification = true;
        }        
        prev_count_millis5 = current_count_millis5;         
      } 
  }  
  
  if(transmit_verification_status == false || receiver_verification_status == false){
    delay(1000);            ................................................
    digitalWrite(LED_connected, LOW);
    digitalWrite(LED_not_connected, HIGH);
    digitalWrite(LED_connecting, LOW);
    identification = false;
  } 
}

void exchange_data_communication_initialization_receive(){
  if(identification == true){
    digitalWrite(LED_connected, HIGH);
    digitalWrite(LED_not_connected, LOW);
    digitalWrite(LED_connecting, LOW);
   
      long int current_count_millis6 = millis();
      long int current_count_millis8 = millis();

      if(current_count_millis6 - prev_count_millis6 >= 3000){
        
        if(twai_receive(&id_receive_108, pdMS_TO_TICKS(2000)) == ESP_OK){
          if(id_receive_108.identifier == 0x108){
            printf("\nID: %X DLC: %d  Data ID: \t", id_receive_108.identifier, id_receive_108.data_length_code);

            for(int i=0; i < id_receive_108.data_length_code; i++){
              printf("%02X\t", id_receive_108.data[i]); 
            }
 
            available_output_current = id_receive_108.data[3];

            available_output_voltage_msb = id_receive_108.data[2]; 
            available_output_voltage_lsb = id_receive_108.data[1];
            available_output_voltage = (available_output_voltage_msb << 8) | available_output_voltage_lsb;

            charging_power = available_output_voltage * available_output_current;
                        
            printf("\nAvailable output voltage DC Station:  %0.2f V \nAvailable output current DC Station:  %0.2f A\n", available_output_voltage, available_output_current);
          }

        } else{
        printf("\nData 108 not available");
        }
        
  
      if(current_count_millis8 - prev_count_millis8 = 2500){

      }
        if(twai_receive(&id_receive_109, pdMS_TO_TICKS(2500)) == ESP_OK){
          if(id_receive_109.identifier == 0x109){
            printf("\nID: %X DLC: %d Data ID: \t", id_receive_109.identifier, id_receive_109.data_length_code);

            for(int i=0; i < id_receive_109.data_length_code; i++){
              printf("%d\t", id_receive_109.data[i]); 
            }
            printf("\n");
            case_109_5 = id_receive_109.data[5];
            switch (case_109_5){
              case 0:
                printf("Station status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");                        
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 0;
                break;
              case 1:
                printf("Station status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 0;               
                break;
              case 2:
                printf("Station status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");               
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 0;               
                break;
              case 3:
                printf("Station status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");               
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 0;                               
                break;
              case 4:
                printf("Station status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");               
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 0;               
                break;
              case 5:
                printf("Station status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");               
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 0;               
                break;
              case 6:
                printf("Station status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");               
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 0;               
                break;
              case 7:
                printf("Station status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");               
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 0;               
                break;
              case 8:
                printf("Station status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");                
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 0;               
                break;
              case 9:
                printf("Station status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n"); 
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 0;              
                break;
              case 10:
                printf("Station status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n"); 
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 0;              
                break;
              case 11:
                printf("Station status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 0;               
                break;
              case 12:
                printf("Station status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");    
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 0;           
                break;
              case 13:
                printf("Station status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");   
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 0;            
                break;
              case 14:
                printf("Station status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 0;               
                break;
              case 15:
                printf("Station status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: OPERATING    \n");
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 0;               
                break;
              case 16:
                printf("Station status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n"); 
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 0;              
                break;
              case 17:
                printf("Station status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n"); 
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 0;              
                break;
              case 18:
                printf("Station status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n"); 
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 0;              
                break;
              case 19:
                printf("Station status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n");   
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 0;            
                break;
              case 20:
                printf("Station status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n");   
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 0;            
                break;
              case 21:
                printf("Station status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n");    
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 0;           
                break;
              case 22:
                printf("Station status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n");  
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 0;                  
                break;
              case 23:
                printf("Station status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n");
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 0;                           
                break;
              case 24:
                printf("Station status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n");
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 0;               
                break;
              case 25:
                printf("Station status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n");
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 0;                 
                break;
              case 26:
                printf("Station status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n"); 
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 0;                
                break;
              case 27:
                printf("Station status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n"); 
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 0;                
                break;
              case 28:
                printf("Station status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n");  
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 0;               
                break;
              case 29:
                printf("Station status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n"); 
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 0;                 
                break;
              case 30:
                printf("Station status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n");
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 0;                  
                break;
              case 31:
                printf("Station status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: OPERATING    \n");  
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 0;                
                break;
              case 32:
                printf("Station status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n");    
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 1;           
                break;
              case 33:
                printf("Station status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n");
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 1;                  
                break;
              case 34:
                printf("Station status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n");   
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 1;               
                break;
              case 35:
                printf("Station status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n");   
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 1;               
                break;
              case 36:
                printf("Station status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n"); 
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 1;                 
                break;
              case 37:
                printf("Station status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n"); 
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 1;                
                break;
              case 38:
                printf("Station status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n");     
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 1;             
                break;
              case 39:
                printf("Station status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n"); 
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 0; charger_stop_control = 1;                 
                break;
              case 40:
                printf("Station status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n");  
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 1;                
                break;
              case 41:
                printf("Station status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n"); 
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 1;               
                break;
              case 42:
                printf("Station status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n");   
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 1;             
                break;
              case 43:
                printf("Station status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n");  
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 1;              
                break;
              case 44:
                printf("Station status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n");  
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 1;              
                break;
              case 45:
                printf("Station status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n");  
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 1;              
                break;
              case 46:
                printf("Station status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n");     
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 1;           
                break;
              case 47:
                printf("Station status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: NORMAL   \nCharger stop control: STOP CHARGING    \n");   
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 0; charger_stop_control = 1;             
                break;
              case 48:
                printf("Station status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n");        
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 1;        
                break;
              case 49:
                printf("Station status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n");        
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 1;              
                break;
              case 50:
                printf("Station status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n"); 
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 1;                     
                break;
              case 51:
                printf("Station status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n");        
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 1;              
                break;
              case 52:
                printf("Station status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n"); 
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 1;                    
                break;
              case 53:
                printf("Station status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n"); 
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 1;                     
                break;
              case 54:
                printf("Station status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n"); 
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 1;                     
                break;
              case 55:
                printf("Station status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: COMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n");      
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 0; charging_system_malfunction = 1; charger_stop_control = 1;                
                break;
              case 56:
                printf("Station status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n");  
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 1;                    
                break;
              case 57:
                printf("Station status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n");               
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 1;    
                break;
              case 58:
                printf("Station status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n");     
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 1;              
                break;
              case 59:
                printf("Station status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: UNCLOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n");  
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 0; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 1;                 
                break;
              case 60:
                printf("Station status: STANDBY  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n"); 
                station_status = 0; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 1;                  
                break;
              case 61:
                printf("Station status: CHARGING  \nStation malfunction: NORMAL \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n");  
                station_status = 1; station_malfunction = 0; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 1;             
                break;
              case 62:
                printf("Station status: STANDBY  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n");
                station_status = 0; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 1;               
                break;
              case 63:
                printf("Station status: CHARGING  \nStation malfunction: FAULT \nVehicle connector lock: LOCKED  \nBattery incompatibility: INCOMPATIBLE   \nCharging system malfunction: MALFUNCTION   \nCharger stop control: STOP CHARGING    \n");
                station_status = 1; station_malfunction = 1; vehicle_connector_lock = 1; battery_incompatibility = 1; charging_system_malfunction = 1; charger_stop_control = 1;               
                break;
            }

            // for condition relay all closed
            if(station_malfunction == 0 && vehicle_connector_lock == 1 && battery_incompatibility == 0 && charging_system_malfunction == 0 && charger_stop_control == 0){
              delay(1000);
              digitalWrite(charger_lock, LOW);
              delay(5000);
              digitalWrite(charger_relay, LOW);
              charger_relay_status = 1; 

              // output voltage & current, using sensor measurement to know value, ini nanti dicoba dipakaikan nilai potensio, dan nanti dihubungkan pakai button   
              // jadi nanti saat potensionya ini puterin apa, baru button langsung, nah untuk output current nanti nyesuain dengan current request


            } 
            // else if(station_malfunction == 1 || vehicle_connector_lock == 0 || battery_incompatibility == 1 || charging_system_malfunction == 1 || charger_stop_control == 1){
            //   digitalWrite(charger_relay, HIGH);
            //   charger_relay_status = 0;
            //   delay(5000);
            //   digitalWrite(charger_lock, HIGH);
            // }

          }
        } else{
        printf("\nData 109 not available");
        }
      }
      prev_count_millis6 = current_count_millis6;         
  }
}

void exchange_data_communication_initialization_transmit(){
  if(receiver_verification_status == true && transmit_verification_status == true){
    operation_id_100();
    operation_id_101();
    operation_id_102();

    start_exchange = true;

    while(start_exchange != false){
      long int current_count_millis7 = millis();

      if(current_count_millis7 - prev_count_millis7 >= 1000){
        message_transfer(&id_transmit_100, 0x100, 8, Status_for_id_100);
        message_transfer(&id_transmit_101, 0x101, 8, Status_for_id_101);
        message_transfer(&id_transmit_102, 0x102, 8, Status_for_id_102);
      }
      prev_count_millis7 = current_count_millis7;         
    }
  }
}


//////////////////////////////////////////////  OTHER  //////////////////////////////////////////////


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
  maximum_charging_time = rated_capacity_of_battery / charging_power;

  int minutes = maximum_charging_time * 60;                                               // Convert hours to minutes
  int seconds = ((maximum_charging_time * 3600) - (minutes * 60)) * 10;                   // Convert remaining to seconds & Multiply by 10 to get the actual seconds  

  minutes = min(minutes, 255);
  seconds = min(seconds / 10, 254);                                                       // divide by 10 because each bit represents 10 seconds
  
  Status_for_id_101[2] = minutes;
  Status_for_id_101[1] = seconds;

  // Print the values to the serial monitor for debugging
  printf("Minutes: %d, Seconds: %d", Status_for_id_101[2],Status_for_id_101[1]);
}


void operation_id_102(){
  target_battery_voltage = 81.11;     // volt
  target_battery_voltage_msb = (target_battery_voltage >> 8) & 0xFF;
  target_battery_voltage_lsb = (target_battery_voltage & 0xFF);


  // for battery temp condition
  if (battery_temp_mapping < battery_temp_heat_target){
    high_battery_temperature = 0;
  } else if (battery_temp_mapping >= battery_temp_heat_target){
    high_battery_temperature = 1;
    if (battery_temp_mapping == battery_temp_overheat){
      printf("BATTERY OVERHEAT! BATTERY OVERHEAT! BATTERY OVERHEAT! BATTERY OVERHEAT! BATTERY OVERHEAT! BATTERY OVERHEAT!\n");
      digitalWrite(charger_relay, HIGH);
      charger_relay_status = 0;
      delay(5000);
      digitalWrite(charger_lock, HIGH);
    }
  } 

  // for charging current request condition
  if (vehicle_charging_enable == 1 && charger_lock == LOW){
    charging_current_request = 20;    //20A penyesuaian power supply yang ada
  } else if (vehicle_charging_enable == 0 && charger_lock == HIGH){
    charging_current_request = 0;
  } else if (high_battery_temperature == 1){
    /*for (int i = charging_current_request; i > currentsetpoint_condition; i -= 0.01){
      delay(100);
    }*/
    charging_current_request = 10;
  } else if (battery_voltage_mapping == target_battery_voltage){
    /*for (int j = charging_current_request; j > currentsetpoint_target; j -= 0.01){
      delay(100);
    }*/
    charging_current_request = 0;
  }


  // condition 102[5]  ************
  // for charging enable condition
  if (charger_stop_control == 0 || battery_incompatibility == 0 || station_malfunction == 0 || charging_system_malfunction == 0 || station_malfunction == 0){
    vehicle_charging_enable == 1;   //enable
  } else if(battery_charger_output_voltage == maximum_battery_voltage || charger_stop_control == 1 || battery_incompatibility == 1 || charging_system_malfunction == 1 || station_malfunction == 1 || battery_temp_mapping == battery_temp_overheat){
    vehicle_charging_enable == 0;   //disable
  }

  
  // for vehicle shift level position     *use button just for try  



  // for charging system fault  *condition when the current not detect or there's no charging 

  
  // for vehicle status
  if (charger_relay_status == 1){
    vehicle_status = 0;
  } else if (charger_relay_status == 0){
    vehicle_status = 1;
  }

  // for normal stop request before charging

  // ************



  // condition 102[4]  ************
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

  // ************

  if (vehicle_charging_enable == 0 && vehicle_shift_lever_position == 0 && charging_system_fault == 0 && vehicle_status == 0 && normal_stop_request_before_charging == 0){
    Status_for_id_102[5] = 0;
  } else if (vehicle_charging_enable == 1 && vehicle_shift_lever_position == 0 && charging_system_fault == 0 && vehicle_status == 0 && normal_stop_request_before_charging == 0){
    Status_for_id_102[5] = 1;
  } else if (vehicle_charging_enable == 0 && vehicle_shift_lever_position == 1 && charging_system_fault == 0 && vehicle_status == 0 && normal_stop_request_before_charging == 0){
    Status_for_id_102[5] = 2;
  } else if (vehicle_charging_enable == 1 && vehicle_shift_lever_position == 1 && charging_system_fault == 0 && vehicle_status == 0 && normal_stop_request_before_charging == 0){
    Status_for_id_102[5] = 3;
  } else if (vehicle_charging_enable == 0 && vehicle_shift_lever_position == 0 && charging_system_fault == 1 && vehicle_status == 0 && normal_stop_request_before_charging == 0){
    Status_for_id_102[5] = 4;
  } else if (vehicle_charging_enable == 1 && vehicle_shift_lever_position == 0 && charging_system_fault == 1 && vehicle_status == 0 && normal_stop_request_before_charging == 0){
    Status_for_id_102[5] = 5;
  } else if (vehicle_charging_enable == 0 && vehicle_shift_lever_position == 1 && charging_system_fault == 1 && vehicle_status == 0 && normal_stop_request_before_charging == 0){
    Status_for_id_102[5] = 6;
  } else if (vehicle_charging_enable == 1 && vehicle_shift_lever_position == 1 && charging_system_fault == 1 && vehicle_status == 0 && normal_stop_request_before_charging == 0){
    Status_for_id_102[5] = 7;
  } else if (vehicle_charging_enable == 0 && vehicle_shift_lever_position == 0 && charging_system_fault == 0 && vehicle_status == 1 && normal_stop_request_before_charging == 0){
    Status_for_id_102[5] = 8;
  } else if (vehicle_charging_enable == 1 && vehicle_shift_lever_position == 0 && charging_system_fault == 0 && vehicle_status == 1 && normal_stop_request_before_charging == 0){
    Status_for_id_102[5] = 9;
  } else if (vehicle_charging_enable == 0 && vehicle_shift_lever_position == 1 && charging_system_fault == 0 && vehicle_status == 1 && normal_stop_request_before_charging == 0){
    Status_for_id_102[5] = 10;
  } else if (vehicle_charging_enable == 1 && vehicle_shift_lever_position == 1 && charging_system_fault == 0 && vehicle_status == 1 && normal_stop_request_before_charging == 0){
    Status_for_id_102[5] = 11;
  } else if (vehicle_charging_enable == 0 && vehicle_shift_lever_position == 0 && charging_system_fault == 1 && vehicle_status == 1 && normal_stop_request_before_charging == 0){
    Status_for_id_102[5] = 12;
  } else if (vehicle_charging_enable == 1 && vehicle_shift_lever_position == 0 && charging_system_fault == 1 && vehicle_status == 1 && normal_stop_request_before_charging == 0){
    Status_for_id_102[5] = 13;
  } else if (vehicle_charging_enable == 0 && vehicle_shift_lever_position == 1 && charging_system_fault == 1 && vehicle_status == 1 && normal_stop_request_before_charging == 0){
    Status_for_id_102[5] = 14;
  } else if (vehicle_charging_enable == 1 && vehicle_shift_lever_position == 1 && charging_system_fault == 1 && vehicle_status == 1 && normal_stop_request_before_charging == 0){
    Status_for_id_102[5] = 15;
  } else if (vehicle_charging_enable == 0 && vehicle_shift_lever_position == 0 && charging_system_fault == 0 && vehicle_status == 0 && normal_stop_request_before_charging == 1){
    Status_for_id_102[5] = 16;
  } else if (vehicle_charging_enable == 1 && vehicle_shift_lever_position == 0 && charging_system_fault == 0 && vehicle_status == 0 && normal_stop_request_before_charging == 1){
    Status_for_id_102[5] = 17;
  } else if (vehicle_charging_enable == 0 && vehicle_shift_lever_position == 1 && charging_system_fault == 0 && vehicle_status == 0 && normal_stop_request_before_charging == 1){
      Status_for_id_102[5] = 18;
  } else if (vehicle_charging_enable == 1 && vehicle_shift_lever_position == 1 && charging_system_fault == 0 && vehicle_status == 0 && normal_stop_request_before_charging == 1){
      Status_for_id_102[5] = 19;
  } else if (vehicle_charging_enable == 0 && vehicle_shift_lever_position == 0 && charging_system_fault == 1 && vehicle_status == 0 && normal_stop_request_before_charging == 1){
      Status_for_id_102[5] = 20;
  } else if (vehicle_charging_enable == 1 && vehicle_shift_lever_position == 0 && charging_system_fault == 1 && vehicle_status == 0 && normal_stop_request_before_charging == 1){
      Status_for_id_102[5] = 21;
  } else if (vehicle_charging_enable == 0 && vehicle_shift_lever_position == 1 && charging_system_fault == 1 && vehicle_status == 0 && normal_stop_request_before_charging == 1){
      Status_for_id_102[5] = 22;
  } else if (vehicle_charging_enable == 1 && vehicle_shift_lever_position == 1 && charging_system_fault == 1 && vehicle_status == 0 && normal_stop_request_before_charging == 1){
      Status_for_id_102[5] = 23;
  } else if (vehicle_charging_enable == 0 && vehicle_shift_lever_position == 0 && charging_system_fault == 0 && vehicle_status == 1 && normal_stop_request_before_charging == 1){
      Status_for_id_102[5] = 24;
  } else if (vehicle_charging_enable == 1 && vehicle_shift_lever_position == 0 && charging_system_fault == 0 && vehicle_status == 1 && normal_stop_request_before_charging == 1){
      Status_for_id_102[5] = 25;
  } else if (vehicle_charging_enable == 0 && vehicle_shift_lever_position == 1 && charging_system_fault == 0 && vehicle_status == 1 && normal_stop_request_before_charging == 1){
      Status_for_id_102[5] = 26;
  } else if (vehicle_charging_enable == 1 && vehicle_shift_lever_position == 1 && charging_system_fault == 0 && vehicle_status == 1 && normal_stop_request_before_charging == 1){
      Status_for_id_102[5] = 27;
  } else if (vehicle_charging_enable == 0 && vehicle_shift_lever_position == 0 && charging_system_fault == 1 && vehicle_status == 1 && normal_stop_request_before_charging == 1){
      Status_for_id_102[5] = 28;
  } else if (vehicle_charging_enable == 1 && vehicle_shift_lever_position == 0 && charging_system_fault == 1 && vehicle_status == 1 && normal_stop_request_before_charging == 1){
      Status_for_id_102[5] = 29;
  } else if (vehicle_charging_enable == 0 && vehicle_shift_lever_position == 1 && charging_system_fault == 1 && vehicle_status == 1 && normal_stop_request_before_charging == 1){
      Status_for_id_102[5] = 30;
  } else if (vehicle_charging_enable == 1 && vehicle_shift_lever_position == 1 && charging_system_fault == 1 && vehicle_status == 1 && normal_stop_request_before_charging == 1){
      Status_for_id_102[5] = 31;
  } else{
      printf("\nError none of them condition in 102.5!");
  }



  if (battery_overvoltage == 0 && battery_undervoltage == 0 && battery_current_deviation_error == 0 && high_battery_temperature == 0 && battery_voltage_deviation_error == 0){
    Status_for_id_102[4] = 0;
  } else if (battery_overvoltage == 1 && battery_undervoltage == 0 && battery_current_deviation_error == 0 && high_battery_temperature == 0 && battery_voltage_deviation_error == 0){
    Status_for_id_102[4] = 1;
  } else if (battery_overvoltage == 0 && battery_undervoltage == 1 && battery_current_deviation_error == 0 && high_battery_temperature == 0 && battery_voltage_deviation_error == 0){
    Status_for_id_102[4] = 2;
  } else if (battery_overvoltage == 1 && battery_undervoltage == 1 && battery_current_deviation_error == 0 && high_battery_temperature == 0 && battery_voltage_deviation_error == 0){
    Status_for_id_102[4] = 3;
  } else if (battery_overvoltage == 0 && battery_undervoltage == 0 && battery_current_deviation_error == 1 && high_battery_temperature == 0 && battery_voltage_deviation_error == 0){
    Status_for_id_102[4] = 4;
  } else if (battery_overvoltage == 1 && battery_undervoltage == 0 && battery_current_deviation_error == 1 && high_battery_temperature == 0 && battery_voltage_deviation_error == 0){
    Status_for_id_102[4] = 5;
  } else if (battery_overvoltage == 0 && battery_undervoltage == 1 && battery_current_deviation_error == 1 && high_battery_temperature == 0 && battery_voltage_deviation_error == 0){
    Status_for_id_102[4] = 6;
  } else if (battery_overvoltage == 1 && battery_undervoltage == 1 && battery_current_deviation_error == 1 && high_battery_temperature == 0 && battery_voltage_deviation_error == 0){
    Status_for_id_102[4] = 7;
  } else if (battery_overvoltage == 0 && battery_undervoltage == 0 && battery_current_deviation_error == 0 && high_battery_temperature == 1 && battery_voltage_deviation_error == 0){
    Status_for_id_102[4] = 8;
  } else if (battery_overvoltage == 1 && battery_undervoltage == 0 && battery_current_deviation_error == 0 && high_battery_temperature == 1 && battery_voltage_deviation_error == 0){
    Status_for_id_102[4] = 9;
  } else if (battery_overvoltage == 0 && battery_undervoltage == 1 && battery_current_deviation_error == 0 && high_battery_temperature == 1 && battery_voltage_deviation_error == 0){
    Status_for_id_102[4] = 10;
  } else if (battery_overvoltage == 1 && battery_undervoltage == 1 && battery_current_deviation_error == 0 && high_battery_temperature == 1 && battery_voltage_deviation_error == 0){
    Status_for_id_102[4] = 11;
  } else if (battery_overvoltage == 0 && battery_undervoltage == 0 && battery_current_deviation_error == 1 && high_battery_temperature == 1 && battery_voltage_deviation_error == 0){
    Status_for_id_102[4] = 12;
  } else if (battery_overvoltage == 1 && battery_undervoltage == 0 && battery_current_deviation_error == 1 && high_battery_temperature == 1 && battery_voltage_deviation_error == 0){
    Status_for_id_102[4] = 13;
  } else if (battery_overvoltage == 0 && battery_undervoltage == 1 && battery_current_deviation_error == 1 && high_battery_temperature == 1 && battery_voltage_deviation_error == 0){
    Status_for_id_102[4] = 14;
  } else if (battery_overvoltage == 1 && battery_undervoltage == 1 && battery_current_deviation_error == 1 && high_battery_temperature == 1 && battery_voltage_deviation_error == 0){
    Status_for_id_102[4] = 15;
  } else if (battery_overvoltage == 0 && battery_undervoltage == 0 && battery_current_deviation_error == 0 && high_battery_temperature == 0 && battery_voltage_deviation_error == 1){
    Status_for_id_102[4] = 16;
  } else if (battery_overvoltage == 1 && battery_undervoltage == 0 && battery_current_deviation_error == 0 && high_battery_temperature == 0 && battery_voltage_deviation_error == 1){
    Status_for_id_102[4] = 17;
  } else if (battery_overvoltage == 0 && battery_undervoltage == 1 && battery_current_deviation_error == 0 && high_battery_temperature == 0 && battery_voltage_deviation_error == 1){
    Status_for_id_102[4] = 18;
  } else if (battery_overvoltage == 1 && battery_undervoltage == 1 && battery_current_deviation_error == 0 && high_battery_temperature == 0 && battery_voltage_deviation_error == 1){
    Status_for_id_102[4] = 19;
  } else if (battery_overvoltage == 0 && battery_undervoltage == 0 && battery_current_deviation_error == 1 && high_battery_temperature == 0 && battery_voltage_deviation_error == 1){
    Status_for_id_102[4] = 20;
  } else if (battery_overvoltage == 1 && battery_undervoltage == 0 && battery_current_deviation_error == 1 && high_battery_temperature == 0 && battery_voltage_deviation_error == 1){
    Status_for_id_102[4] = 21;
  } else if (battery_overvoltage == 0 && battery_undervoltage == 1 && battery_current_deviation_error == 1 && high_battery_temperature == 0 && battery_voltage_deviation_error == 1){
    Status_for_id_102[4] = 22;
  } else if (battery_overvoltage == 1 && battery_undervoltage == 1 && battery_current_deviation_error == 1 && high_battery_temperature == 0 && battery_voltage_deviation_error == 1){
    Status_for_id_102[4] = 23;
  } else if (battery_overvoltage == 0 && battery_undervoltage == 0 && battery_current_deviation_error == 0 && high_battery_temperature == 1 && battery_voltage_deviation_error == 1){
    Status_for_id_102[4] = 24;
  } else if (battery_overvoltage == 1 && battery_undervoltage == 0 && battery_current_deviation_error == 0 && high_battery_temperature == 1 && battery_voltage_deviation_error == 1){
    Status_for_id_102[4] = 25;
  } else if (battery_overvoltage == 0 && battery_undervoltage == 1 && battery_current_deviation_error == 0 && high_battery_temperature == 1 && battery_voltage_deviation_error == 1){
    Status_for_id_102[4] = 26;
  } else if (battery_overvoltage == 1 && battery_undervoltage == 1 && battery_current_deviation_error == 0 && high_battery_temperature == 1 && battery_voltage_deviation_error == 1){
    Status_for_id_102[4] = 27;
  } else if (battery_overvoltage == 0 && battery_undervoltage == 0 && battery_current_deviation_error == 1 && high_battery_temperature == 1 && battery_voltage_deviation_error == 1){
    Status_for_id_102[4] = 28;
  } else if (battery_overvoltage == 1 && battery_undervoltage == 0 && battery_current_deviation_error == 1 && high_battery_temperature == 1 && battery_voltage_deviation_error == 1){
    Status_for_id_102[4] = 29;
  } else if (battery_overvoltage == 0 && battery_undervoltage == 1 && battery_current_deviation_error == 1 && high_battery_temperature == 1 && battery_voltage_deviation_error == 1){
    Status_for_id_102[4] = 30;
  } else if (battery_overvoltage == 1 && battery_undervoltage == 1 && battery_current_deviation_error == 1 && high_battery_temperature == 1 && battery_voltage_deviation_error == 1){
    Status_for_id_102[4] = 31;
  } else{
    printf("\nError none of them condition in 102.4!");
  }


  Status_for_id_102[3] = charging_current_request;
  Status_for_id_102[2] = rated_capacity_of_battery_msb; 
  Status_for_id_102[1] = rated_capacity_of_battery_lsb;
  Status_for_id_102[0] = 25;                                                          // protocol number


}

/*
void remaining_charging_timer(){
  long previousMillis = 0;
  const long interval = 10000;         // Interval of 10 seconds

  long seconds = 0;
  long minutes = 0;
  long hours = 0;

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    seconds+=10;

    if (seconds >= 60) {
      seconds = 0;
      minutes++;

      if (minutes >= 60) {
        minutes = 0;
        hours++;
      }
    }

    Status_for_id_109[6] = seconds;
    Status_for_id_109[7] = minutes;

    // Print the time in HH:MM:SS format
    Serial.print(hours < 10 ? "0" : "");
    Serial.print(hours);
    Serial.print(":");
    Serial.print(minutes < 10 ? "0" : "");
    Serial.print(minutes);
    Serial.print(":");
    Serial.print(seconds < 10 ? "0" : "");
    Serial.println(seconds);
  }
}
*/


////////////////////////////////////////////// TWAI TRANSCEIVER  //////////////////////////////////////////////


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
  if (twai_transmit(message, pdMS_TO_TICKS(100)) == ESP_OK) {
    printf("Message queued for transmission\n");
  } else {
    printf("Failed to send message");
  }
}



//////////////////////////////////////////////  SET UP  //////////////////////////////////////////////


void setup() {
  Serial.begin(115200);
  twai_setup_and_install();

  printf("================================  Program Start!  ================================\n");
 
  
  pinMode(LED_not_connected, OUTPUT); digitalWrite(LED_not_connected, HIGH);
  pinMode(LED_connecting, OUTPUT); digitalWrite(LED_connecting, LOW);
  pinMode(LED_connected, OUTPUT); digitalWrite(LED_connected, LOW);

  pinMode(charger_lock, OUTPUT);
  pinMode(charger_relay, OUTPUT);
}

void loop(){
 battery_current_voltage_temp();
 identification_chargecontroller_chargeroffboard();
 exchange_data_communication_initialization_receive();
//  exchange_data_communication_initialization_transmit();
}