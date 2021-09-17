/*
BMS CODE FOR EVAM

Designed to run on an Arduino Nano (ARDUINO_AVR_NANO)
Relays certain information from the Battery's internal BMS CAN Bus to the EVAM CAN Bus

!This code is not millis() overflow protected!
*/

#include <SPI.h>
#include <mcp2515.h>
#define DEBUG


struct can_frame canMsg;
struct can_frame batteryMsg;
struct can_frame canStatusMsg;  //status of the node
MCP2515 batteryMcp2515(10);
MCP2515 evamMcp2515(9);

//constants for printing battery values
#ifdef DEBUG
float V, I;
int SOC, Th;
#endif  //DEBUG

#define BMS_CAN_TIMEOUT 1000  //timeout in ms before raising an error, if no message is received on the battery canbus
#define MSG_INTERVAL 100  //timing delay in ms between messages sent by node

unsigned long lastRcv1Millis = 0;
unsigned long lastRcv2Millis = 0;
unsigned long lastSendMillis = 0;
uint8_t errorState = 255;  //state of the node. 0: error, 1: ok, 255: offline

//to update CANBus on the status of the node
void sendStatus(uint8_t status = 0){
  errorState = status;
  #ifdef DEBUG
  Serial.print("Node status: ");
  Serial.println(errorState);
  #endif //DEBUG
  canStatusMsg.data[0] = status;
  evamMcp2515.sendMessage(&canStatusMsg);
}

void sendCanMessage(){
  
  evamMcp2515.sendMessage(&batteryMsg);
  #ifdef DEBUG
  V = (batteryMsg.data[1] * 256.0 + batteryMsg.data[0]) * 0.1;
  I = (batteryMsg.data[3] * 256.0 + batteryMsg.data[2]) * 0.1 - 320.0;
  SOC = batteryMsg.data[6];
  Th = batteryMsg.data[7] - 40 ;
  Serial.println("V = " + String(V) + " | " + "I = " + String(I) +  " | " + "SOC = " + String(SOC)+ " | " + "Th = " + String(Th));
  #endif  //DEBUG
}

void setup() {
  #ifdef DEBUG
  Serial.begin(115200);
  #ifndef ARDUINO_AVR_NANO
  Serial.print("WARNING: This sketch was designed for an arduino Nano");
  #endif //#ifndef ARDUINO_AVR_NANO
  #endif  //DEBUG

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

  //initialise both CAN Bus modules
  batteryMcp2515.reset();
  batteryMcp2515.setBitrate(CAN_500KBPS);
  batteryMcp2515.setNormalMode();

  evamMcp2515.reset();
  evamMcp2515.setBitrate(CAN_500KBPS);
  evamMcp2515.setNormalMode();

  sendStatus(1);  //update status to `ok`
}

void loop() {
  if (batteryMcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if(canMsg.can_id == 2566001651){ //BMS 2
      for(int i = 0; i < 8; i++){
        batteryMsg.data[i]  = canMsg.data[i];
      }
    lastRcv1Millis = millis();

    }
    
    if(canMsg.can_id == 2566002163){ //BMS4
      batteryMsg.data[7] = canMsg.data[0];  //highest cell temperature
      lastRcv2Millis = millis();
    }   
  }
  if ((errorState == 0) && (millis() - lastRcv1Millis < BMS_CAN_TIMEOUT) && (millis() - lastRcv1Millis < BMS_CAN_TIMEOUT)){
    sendStatus(1);  //error is solved
  }
  if (millis() - lastSendMillis > MSG_INTERVAL){
    sendCanMessage(); 
    lastSendMillis = millis();
  }
  if ((errorState != 0) && ((millis() - lastRcv1Millis > BMS_CAN_TIMEOUT) || (millis() - lastRcv1Millis > BMS_CAN_TIMEOUT))){
    sendStatus(0);  //raise error (BMS CAN Timeout)
  }
}
