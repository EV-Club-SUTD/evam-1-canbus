/*
SAS CODE FOR EVAM

FUNCTIONS
- Outputs the steering angle

TODO: 
Code is still uder development
- test SAS sensor porperly
- implement readSteeringAngle()

Designed to run on an Arduino Nano (ARDUINO_AVR_NANO)

!This code is not millis() overflow protected!
*/


#include <SPI.h>
#include <mcp2515.h>
#define DEBUG

//timing stuff
const unsigned long messageInterval = 10;  //timing delay in ms between messages sent by node
unsigned long lastMessageTime = 0;  //keeps track of the timestamp of the last message sent

//steering angle values
uint16_t steeringAngle = 0;  //for CAN message. Split into 2 bytes
uint16_t steeringRawAvg;


//can bus stuff

struct can_frame canMsg;

MCP2515 mcp2515(10);

void sendCanMessage(){
  canMsg.data[0] = steeringAngle >> 8; //MSB, rightshift 8 bits ( byte)
  canMsg.data[1] = steeringAngle & 0x00FF; //LSB, mask in only the lowest byte
  mcp2515.sendMessage(&canMsg);
  
  #ifdef DEBUG  //print brake and throttle values
  Serial.print("Steering Angle = ");
  Serial.println((int16_t)steeringAngle/10 - 1800);

  #endif
}

void readSteering(){
  steeringAngle = 0;  //from sensor
  //TODO
}

void average(){

}

void setup() {
  canMsg.can_id  = 0x020;
  canMsg.can_dlc = 8;
  canMsg.data[0] = 0x00;
  canMsg.data[1] = 0x00;
  canMsg.data[2] = 0x00;
  canMsg.data[3] = 0x00;
  canMsg.data[4] = 0x00;
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;
  
  #ifdef DEBUG  //debug mode
  //while (!Serial);
  Serial.begin(115200);
  Serial.println("SAS");
  #endif
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

}

void loop() {
  readSteering();
  //average();
  if(millis() - lastMessageTime >= messageInterval){
    sendCanMessage();
  }
 
}
