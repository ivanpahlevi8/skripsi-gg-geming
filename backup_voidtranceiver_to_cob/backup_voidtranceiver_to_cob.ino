void information_cob_cc(void * argument){

  for(;;){
    long int current_count_millis5 = millis();
      
    if(current_count_millis5 - prev_count_millis5 >= 1000){  

      // connect/disconnect information (data masuk)
      if(!transceiver_to_cob){
        if(twai_receive(&message_for_receiver, pdMS_TO_TICKS(1000)) == ESP_OK){        
          if(message_for_receiver.identifier == 0x00F12345){
            printf("\nID: %X DLC: %d \nData charger-controller & charger off-board CONNECT!\n", message_for_receiver.identifier, message_for_receiver.data_length_code);
            
            ccs_cob_information_header[0] = 11;
            message_transfer(&message_for_transmitter, 0x00F23456, 8, ccs_cob_information_header);
            
            transceiver_to_cob = true;
          }
        } else{
          printf("Data charger-controller & charger off-board UNCONNECT!");
          
          ccs_cob_information_header[0] = 00;    
          message_transfer(&message_for_transmitter, 0x00F23456, 8, ccs_cob_information_header);
          
          transceiver_to_cob = false;
        }
      } 

      // transmit battery information - voltage - current - temperature
      while(transceiver_to_cob == true){
        CC_Voltage_Current_value[1] = battery_charger_output_voltage_lsb;
        CC_Voltage_Current_value[2] = battery_charger_output_voltage_msb;
        CC_Voltage_Current_value[3] = battery_charger_output_current_lsb;
        CC_Voltage_Current_value[4] = battery_charger_output_current_msb;
        CC_Temperature_value[1] = battery_charger_temp;

        message_transfer(&message_battery_current_voltage_to_cob, 0x18FD15FE, 8, CC_Voltage_Current_value);
        message_transfer(&message_battery_temperature_value_to_cob, 0x18FE50FE, 8, CC_Temperature_value);
      }
      prev_count_millis5 = current_count_millis5;         
    }
  }  
}