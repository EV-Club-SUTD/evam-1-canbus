/*
TPS CODE FOR EVAM

by Nigel Gomes (https://github.com/yik3z)

FUNCTIONS:
-Read throttle positions sensors and publish to CAN Bus
--Possibly average a few readings for stability
-If 2 TPS are connected, check between both sensors to identify throttle faults


-Read brake pressure sensor(s) and publish [pressure inforamtion to CAN Bus
-Convert pressure information to equivalent brake pedal position and publish to CANB us


Designed to run on an Arduino Nano (ARDUINO_AVR_NANO)

!This code is not millis() overflow protected!

TPS functionality is working
BRAKE POSITION VALUE IS SET TO EQUAL TO THROTTLE VALUE FOR TESTING
TODO:  Add in BPS calibration code (things to do are marked with "TODO" in the comments beside it)

NOTE:
** Due to noise on the 5V line when the motors are running, the throttle values between both sensors may differ significantly. This will throw an "ERROR! Throttle mismatch" message in the serial monitor.
** The dual throttle functionality has been diabled by commenting out '#define DUAL_TPS' in TpsConfig.h
** Another way to do it is to raise the 'THROTTLE_DIFFERENCE_LIMIT' in TpsConfig.h to the max differnence you see in the serial monitor

*/

#include "Arduino.h"
#include "TpsConfig.h"

// #define DEBUG

#ifdef DEBUG
void printAccMessage(){
  Serial.print("Throttle = ");
  Serial.print(canAccMsg.data[0]);
  Serial.print("| Brake Pressure = ");
  Serial.print(canAccMsg.data[2]*4);
  Serial.print("| Brake Percent = ");
  Serial.println(canAccMsg.data[4]);
}
#endif 

//reads the raw value from the throttle and converts it to parcentage for the CAN bus
void readFilterThrottle(){
  uint16_t throttleRaw = analogRead(ACC_PIN);                                           //reads 10bit ADC value
  throttleRaw = (checkForErroneousValues(throttleRaw, TPS1_MIN_VAL, TPS1_MAX_VAL))<<4;  //check for problematic values, then leftshift to 14 bit
  //Serial.println(throttleRaw);
  #ifdef DUAL_TPS
    uint16_t throttleRaw2 = analogRead(ACC_PIN2);                                         //reads 10bit ADC value
    //Serial.println(String(throttleRaw) + " | " + String(throttleRaw2));
    throttleRaw2 = (checkForErroneousValues(throttleRaw2, TPS2_MIN_VAL, TPS2_MAX_VAL))<<4;  //check for problematic values, then leftshift to 14 bit
  #endif  //DUAL_TPS

  /* FILTERING */
  #ifdef FILTER_THROTTLE
    uint32_t filteredThrottle = throttleFilter.filter(throttleRaw); //is actually a 16 bit number
    #ifdef DUAL_TPS
      uint32_t filteredThrottle2 = throttleFilter.filter(throttleRaw2); //is actually a 16 bit number
    #endif  //DUAL_TPS

  #else //ndef FILTER_VALUES
    uint32_t filteredThrottle = throttleRaw;
    #ifdef DUAL_TPS
      uint32_t filteredThrottle2 = throttleRaw2;
    #endif  //DUAL_TPS

  #endif  //FILTER_VALUES

  /* MAPPING */
  filteredThrottle = map(filteredThrottle, TPS1_MIN_VAL<<4, TPS1_MAX_VAL<<4, 0, 16000);
  #ifdef DUAL_TPS
    //TODO: implement dual TPS checking code
    #ifdef TPS2_REVERSE_VOLTAGE
      filteredThrottle2 = map(filteredThrottle2, TPS2_MIN_VAL<<4, TPS2_MAX_VAL<<4, 16000, 0);
      
    #else //TPS2 is not reverse voltage
      filteredThrottle2 = map(filteredThrottle2, TPS2_MIN_VAL<<4, TPS2_MAX_VAL<<4, 0, 16000);
    #endif  //TPS2_REVERSE_VOLTAGE

    //DUAL TPS REDUNDANCY CHECK
    int16_t diff = filteredThrottle - filteredThrottle2;
    #ifdef DEBUG
    //Serial.println(String(filteredThrottle) + " | " + String(filteredThrottle2)); //disabled for speeeed hehe
    #endif //DEBUG
    if((diff>THROTTLE_DIFFERENCE_LIMIT) || (diff<-THROTTLE_DIFFERENCE_LIMIT)){  //determined empirically. It may be different based on use cases
      //ERROR!!
      #ifdef DEBUG
      Serial.println("ERROR! Throttle Mismatch!");
      Serial.println(diff);
      #endif
      if(errorState != 0){
        sendStatus(0);
      }
      filteredThrottle = 0; //set throttle to 0 for safety
    }
    else{ //no throttle mismatch
      if(errorState == 0){  //Recovered from throttle mismatch
        sendStatus(1);
      }
      //do any further post processing with the valid throttle data
    }

  #else //not DUAL_TPS
  //??
  #endif  //DUAL_TPS
  //Serial.println(filteredThrottle);
  canAccMsg.data[0] = filteredThrottle>>6;  //rightshift 14 bit back to 8 bit number to send on canbus
}

void readFilterBrake(){
  //uint16_t brakeRaw = analogRead(BRAKE_PIN)<<4;  //reads 10bit ADC value, converts to 14bit //TODO: enable this and disable the next line when the brake sensor is fitted in
  uint16_t brakeRaw = 0;  //dummy data since there's no sensor connected

  /* FILTERING */
  #ifdef FILTER_BRAKE
  uint32_t filteredBrake = brakeFilter.filter(brakeRaw); //is actually a 16 bit number
  #else
  uint32_t filteredBrake = brakeRaw;
  #endif //FILTER_BRAKE
  
  uint8_t brakePercent = calcBrakePercent(filteredBrake>>6);
  canAccMsg.data[2] = (filteredBrake>>6)*2;  //convert back to 8 bit number to send on canbus
  canAccMsg.data[4] = brakePercent;
  //canAccMsg.data[4] = canAccMsg.data[0];  //set brake percent same as throttle percent for testing
}

uint16_t checkForErroneousValues(uint16_t _rawVal, uint16_t minVal, uint16_t maxVal){
  //to catch overflow (0-50)
  uint16_t minVal2 = 50;
  if(minVal > 50){
    minVal2 = minVal;
  }
  if(_rawVal < (minVal2-50)){  //value too low, probably disconnected sensor!
    #ifdef DEBUG
    Serial.print("Sensor Value too low! Is it disconnected?");
    #endif
    if(errorState == 1){  //send CAN error message
      sendStatus(0);
    }
  }else if(_rawVal > (maxVal+50)){ //Value too high, idk what could cause this
    #ifdef DEBUG
    Serial.print("Sensor Value too high!");
    #endif
    if(errorState == 1){  //send CAN error message
      sendStatus(0);
    }
    _rawVal = 0;  //set to 0 for safety
  }else if(_rawVal < minVal){
    _rawVal = minVal; //constrain it to the minimum value
  }else if(_rawVal > maxVal){
    _rawVal = maxVal; //constrain it to the maximum value
  }
  return _rawVal;
}

//calculates the estimated braking power applied by the driver
uint8_t calcBrakePercent(uint8_t _brakeRaw){
  uint8_t _brakePercent = _brakeRaw;  //TODO: convert from sensor reading to brake percentage
  return _brakePercent;
}

void sendStatus(uint8_t status){
  errorState = status;
  #ifdef DEBUG
  Serial.print("Node status: ");
  Serial.println(errorState);
  #endif //DEBUG
  canStatusMsg.data[0] = status;
  mcp2515.sendMessage(&canStatusMsg);
}

void setupCan(){
  //status message
  canStatusMsg.can_id  = TPS_STATUS_MSG_ID;
  canStatusMsg.can_dlc = 1;
  canStatusMsg.data[0] = errorState;
  
  //accelerator n brake message
  canAccMsg.can_id  = THROTTLE_BRAKE_MSG_ID;
  canAccMsg.can_dlc = 5;
  canAccMsg.data[0] = 0x00;
  canAccMsg.data[1] = 0x00;
  canAccMsg.data[2] = 0x00;
  #ifdef TWO_BRAKE_SENSORS
  canAccMsg.data[3] = 0x00; //set to 0 becuase 
  #else
  canAccMsg.data[3] = 0xFF; //set brake 2 to 255 to flag that it is not in use
  #endif
  canAccMsg.data[4] = 0x00;

  //initialise CAN Bus module
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

void readIncomingMessages(){
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if(canMsg.can_id == NODE_STATUS_REQUEST_MSG_ID){ //Node Status Request Message ID
      sendStatus(errorState);
      #ifdef DEBUG
      Serial.println("Node Status Requested");
      #endif  //DEBUG
    }
  }
}
 
void setup() {
  #ifdef DEBUG  //debug mode
  Serial.begin(115200);
  Serial.println("TPS Node");
  //Serial.println("WARNING: Brake VALUE SET TO ACCELERATOR VALUE FOR TESTING");
  #ifndef ARDUINO_AVR_NANO
  Serial.println("WARNING: This sketch was designed for an arduino Nano");
  #endif //#ifndef ARDUINO_AVR_NANO
  #endif //#ifdef DEBUG
  setupCan();
  sendStatus(1);
}

void loop() {
  readIncomingMessages();
  readFilterThrottle();
  readFilterBrake();
  if(millis() - lastMessageTime >= MSG_INTERVAL){
    mcp2515.sendMessage(&canAccMsg);

    lastMessageTime = millis();
  }

  #ifdef DEBUG  
  if (millis() - lastPrintMillis >= PRINT_INTERVAL)
  {
    printAccMessage();      //print brake and throttle values
    lastPrintMillis = millis();
  }
  #endif  
}
