/*
BMS CODE FOR EVAM

FUNCTIONS
-Relay certain information from the Battery's internal BMS CAN Bus to the EVAM CAN Bus
-Monitor the voltages of the 5V and 12V rail and publish to CANBus

TODO: 
CODE **SHOULD** WORK, BUT IS PENDING TESTING

Designed to run on an Arduino Nano (ARDUINO_AVR_NANO)

!This code is not millis() overflow protected!
*/

#include "Arduino.h"
#include <SPI.h>
#include <mcp2515.h>
#include <EwmaT.h>
// #define DEBUG

//can bus
struct can_frame canMsg;        //generic CAN message for recieving data
struct can_frame batteryMsg;
struct can_frame canStatusMsg;  //status of the node
struct can_frame voltageMsg;  //Voltage Rail values
MCP2515 batteryMcp2515(10);
MCP2515 evamMcp2515(9);

//constants for printing battery values
#ifdef DEBUG
uint16_t V, I;
uint16_t SOC, Th;
#endif  //DEBUG

//timings
#define BMS_CAN_TIMEOUT 1000  //timeout in ms before raising an error, if no message is received on the battery canbus
#define BATT_MSG_INTERVAL 100  //timing delay in ms between battery messages sent by node
#define VOLTAGE_MSG_INTERVAL 500  //timing delay in ms between rail voltage messages sent by node

#define NODE_STATUS_REQUEST_MSG_ID  7

unsigned long lastRcv1Millis = 0;
unsigned long lastRcv2Millis = 0;
unsigned long lastBattSendMillis = 0;
unsigned long lastVoltageSendMillis = 0;
uint8_t errorState = 255;  //state of the node. 0: BMS can timeout, 1: ok, 255: offline

//voltage sensing
#define SENSE_5V A0
#define SENSE_12V A1
uint16_t filtered5v = 0;
uint16_t filtered12v = 0;
EwmaT <uint32_t> filter5v(3, 100);
EwmaT <uint32_t> filter12v(3, 100);

/***CAN BUS MESSAGE FUNCTIONS***/

//to update CANBus on the status of the node
void sendStatus(uint8_t status = errorState){
  errorState = status;
  #ifdef DEBUG
  Serial.print("Node status: ");
  Serial.println(errorState);
  #endif //DEBUG
  canStatusMsg.data[0] = status;
  evamMcp2515.sendMessage(&canStatusMsg);
}

void sendBattMessage(){
  evamMcp2515.sendMessage(&batteryMsg);
  #ifdef DEBUG
  //sorry all nmumbers displayed here in integers for performance. Use the CAN Bus data for higher res
  V = ((batteryMsg.data[1] << 8) | batteryMsg.data[0]) /10;
  I = ((batteryMsg.data[3] << 8) | batteryMsg.data[2]) /10 - 320;
  SOC = batteryMsg.data[6];
  Th = batteryMsg.data[7] - 40 ;
  Serial.println("V = " + String(V) + " | " + "I = " + String(I) +  " | " + "SOC = " + String(SOC)+ " | " + "Th = " + String(Th));
  #endif  //DEBUG
}

void sendVoltageMessage(){
  evamMcp2515.sendMessage(&voltageMsg);
  #ifdef DEBUG
  uint8_t v5 = voltageMsg.data[0]/36;
  uint8_t v5dec = (voltageMsg.data[0]%36)*5;
  uint8_t v12 = voltageMsg.data[2]/10;
  uint8_t v12dec = (voltageMsg.data[0]%10)*5;
  //NO USING FLOATS!!
  Serial.println("5V = " + String(v5) + ((v5dec<10) ? (".0") : (".")) + String(v5dec) + " | " + "12V = " + String(v12) + ((v12dec<10) ? (".0") : (".")) + String(v12dec));
  #endif  //DEBUG
}

/***OTHERS***/
void readFilterVoltages(){
  //analogRead values promoted to 14 bit since it's filtered xd

  uint16_t raw5v = analogRead(SENSE_5V)<<4;  //reads 10bit ADC value, converts to 14bit. might need to multiply by a scaling factor
  uint16_t raw12v = analogRead(SENSE_12V)<<4;
  uint32_t filtered5v = filter5v.filter(raw5v); //is actually a 16 bit number
  uint32_t filtered12v = filter12v.filter(raw12v);
  voltageMsg.data[0] = filtered5v>>6;
  voltageMsg.data[2] = filtered12v>>6;
}

void setup() {
  #ifdef DEBUG
  Serial.begin(115200);
  Serial.println("BMS Node");
  #ifndef ARDUINO_AVR_NANO
  Serial.println("WARNING: This sketch was designed for an arduino Nano");
  #endif //#ifndef ARDUINO_AVR_NANO
  #endif  //DEBUG

  //set up voltage sense pins
  analogReference(EXTERNAL); //AREF is connected to 3v3 pin


  //status message
  canStatusMsg.can_id  = 0x09;
  canStatusMsg.can_dlc = 1;
  canStatusMsg.data[0] = errorState;

  //main message
  batteryMsg.can_id  = 0x24;
  batteryMsg.can_dlc = 8;
  batteryMsg.data[0] = 0x00;
  batteryMsg.data[1] = 0x00;
  batteryMsg.data[2] = 0x00;
  batteryMsg.data[3] = 0x00;
  batteryMsg.data[4] = 0x00;
  batteryMsg.data[5] = 0x00;
  batteryMsg.data[6] = 0x00;
  batteryMsg.data[7] = 0x00;

  //Voltage Rail values message
  voltageMsg.can_id  = 37;
  voltageMsg.can_dlc = 2;
  voltageMsg.data[0] = 0;
  voltageMsg.data[0] = 0;

  //initialise both CAN Bus modules
  batteryMcp2515.reset();
  batteryMcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);
  batteryMcp2515.setNormalMode();

  evamMcp2515.reset();
  evamMcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  evamMcp2515.setNormalMode();
  while((millis() - lastRcv1Millis > BMS_CAN_TIMEOUT) || (millis() - lastRcv1Millis > BMS_CAN_TIMEOUT)){
    //wait for first messages to come in
  }
  sendStatus(1); //node is ready
}

void loop() {
  if (batteryMcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if(canMsg.can_id == 2566001651){ //BMS 2
      for(int i = 0; i < 7; i++){
        batteryMsg.data[i]  = canMsg.data[i];
      }
    lastRcv1Millis = millis();
    }
    
    if(canMsg.can_id == 2566002163){ //BMS4
      batteryMsg.data[7] = canMsg.data[0];  //highest cell temperature
      lastRcv2Millis = millis();
    }
  }

  if (evamMcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if(canMsg.can_id == NODE_STATUS_REQUEST_MSG_ID){ //Node Status Request Message ID
      sendStatus();
      Serial.println(errorState);
      //#ifdef DEBUG
      //Serial.println("Node Status Requested");
      //#endif  //DEBUG
    }
  }

  //sample the voltage rails
  readFilterVoltages();

  if (millis() - lastBattSendMillis > BATT_MSG_INTERVAL){
    sendBattMessage(); 
    lastBattSendMillis = millis();
  }
  if (millis() - lastVoltageSendMillis > VOLTAGE_MSG_INTERVAL){
    sendVoltageMessage(); 
    lastVoltageSendMillis = millis();
  }
  
  //message timeouts
  if ((errorState != 0) && ((millis() - lastRcv1Millis > BMS_CAN_TIMEOUT) || (millis() - lastRcv1Millis > BMS_CAN_TIMEOUT))){
    sendStatus(0);  //raise error (BMS CAN Timeout)
    #ifdef DEBUG
    Serial.print("BMS CAN Timeout!!");
    #endif
  }
  if ((errorState == 0) && (millis() - lastRcv1Millis < BMS_CAN_TIMEOUT) && (millis() - lastRcv1Millis < BMS_CAN_TIMEOUT)){
    sendStatus(1);  //error is solved
    #ifdef DEBUG
    Serial.print("BMS CAN Working Again");
    #endif
  }
}
