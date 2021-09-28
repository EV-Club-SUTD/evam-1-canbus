/*
TPS CODE FOR EVAM


FUNCTIONS:
-Read throttle positions sensors and publish to CAN Bus
--Possibly average a few readings for stability
-If 2 TPS are connected, check between both sensors to identify throttle faults

-Read brake pressure sensor(s) and publish [pressure inforamtion to CAN Bus
-Convert pressure information to equivalent brake pedal position and publish to CANB us


Designed to run on an Arduino Nano (ARDUINO_AVR_NANO)
Connect A0 to the throttle 
Connect A1 to the brake

!This code is not millis() overflow protected!
*/

//TODO:  Add in dual TPS and BPS code

#include "Arduino.h"
#include <EwmaT.h>
#include <SPI.h>
#include <mcp2515.h>  //arduino-mcp2515 by autowp: https://github.com/autowp/arduino-mcp2515/
#define DEBUG

//timing stuff
#define MSG_INTERVAL 10  //timing delay in ms between messages sent by node
unsigned long lastMessageTime = 0;  //keeps track of the timestamp of the last message sent

//node status
uint8_t errorState = 255;  //state of the node. 0: error, 1: ok, 255: offline


//throttle & brake 
//#define DUAL_TPS_SENSOR //for use with dual TPS (for redundancy)
  #ifdef DUAL_TPS_SENSOR
  //#define TPS2_REVERSE_VOLTAGE  //TPS 2 voltage decreases as the pedal is pressed more
  #endif
#endif
//#define TWO_BRAKE_SENSORS

#define ACC_PIN A0  //analog pin that the accelerator is connected to
#ifdef DUAL_TPS_SENSOR
#define ACC_PIN2 A1
#endif  //DUAL_TPS_SENSOR
#define BRAKE_PIN A2  //analog pin that the brake pressure sensor is connected to
#ifdef TWO_BRAKE_SENSORS  //for front and rear brake circuit pressure monitoring
#define BRAKE_PIN2 A3
#endif  //TWO_BRAKE_SENSORS

EwmaT <uint32_t> throttleFilter(3, 100);
EwmaT <uint32_t> brakeFilter(3, 100);
#ifdef TWO_BRAKE_SENSORS
EwmaT <uint32_t> brakeFilter2(3, 100);
#endif

/***CAN BUS STUFF***/
MCP2515 mcp2515(10);
struct can_frame canStatusMsg;  //status of the node
struct can_frame canAccMsg; //main accelerator/brake message

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

void readFilterThrottle(){
  uint16_t throttleRaw = analogRead(ACC_PIN)<<4;  //reads 10bit ADC value, converts to 14bit

  #ifdef DUAL_TPS_SENSOR
  uint16_t throttleRaw2 = analogRead(ACC_PIN2)<<4;
  
  //TODO: implement dual TPS checking code
  #ifdef TPS2_REVERSE_VOLTAGE
  //??
  #else
  //??
  #endif  //TPS2_REVERSE_VOLTAGE
  #endif  //DUAL_TPS_SENSOR
  
  //filtering
  uint32_t filteredThrottle = throttleFilter.filter(throttleRaw); //is actually a 16 bit number
  canAccMsg.data[0] = filteredThrottle>>6;  //convert back to 8 bit number to send on canbus
}

void readFilterBrake(){
  //uint16_t brakeRaw = analogRead(BRAKE_PIN)<<4;  //reads 10bit ADC value, converts to 14bit
  uint16_t brakeRaw = 0;  //from sensor

    //filtering
  uint32_t filteredBrake = brakeFilter.filter(brakeRaw); //is actually a 16 bit number
  uint8_t brakePercent = calcBrakePercent((filteredBrake>>6)));
  canAccMsg.data[2] = (filteredBrake>>6)*2;  //convert back to 8 bit number to send on canbus
  canAccMsg.data[2] = brakePercent;
}

//calculates the estimated braking power applied by the driver
uint8_t calcBrakePercent(uint8_t brakeRaw){
  uint8_t brakePercent = brakeRaw;  //idk man need a way to convert from sensor reading to brake percentage
  return brakePercent;
}

//to update CANBus on the status of the node
void sendStatus(uint8_t status = 0){
  errorState = status;
  #ifdef DEBUG
  Serial.print("Node status: ");
  Serial.println(errorState);
  #endif //DEBUG
  canStatusMsg.data[0] = status;
  mcp2515.sendMessage(&canStatusMsg);
}
void setup() {
  #ifdef DEBUG  //debug mode
  Serial.begin(115200);
  Serial.println("TPS Node");
  #ifndef ARDUINO_AVR_NANO
  Serial.println("WARNING: This sketch was designed for an arduino Nano");
  #endif //#ifndef ARDUINO_AVR_NANO
  #endif //#ifdef DEBUG

  //status message
  canStatusMsg.can_id  = 0x0A;
  canStatusMsg.can_dlc = 1;
  canStatusMsg.data[0] = errorState;
  
  //accelerator n brake message
  canAccMsg.can_id  = 0x20;
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
  sendStatus(1);  
}

void loop() {
  readFilterThrottle();
  readFilterBrake();
  if(millis() - lastMessageTime >= MSG_INTERVAL){
    mcp2515.sendMessage(&canAccMsg);
    #ifdef DEBUG  
    //print brake and throttle values
    printAccMessage();
    #endif 
  }
}
