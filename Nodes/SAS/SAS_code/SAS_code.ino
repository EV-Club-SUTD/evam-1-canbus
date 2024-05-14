/*
SAS CODE FOR EVAM

by Nigel Gomes (https://github.com/yik3z)

FUNCTIONS
- Outputs the steering angle
- calibrates steering
  - min/max angle
  - center position

Designed to run on an Arduino Nano (ARDUINO_AVR_NANO)

!This code is not millis() overflow protected!
*/

#include "Arduino.h"
#include <EEPROM.h>
#include <SPI.h>
#include <mcp2515.h>
#include <Wire.h>
#include <AS5600.h> //Seeed Library for AS5600:https://github.com/Seeed-Studio/Seeed_Arduino_AS5600
#include "can_ids.h"
#define DEBUG

//timing stuff
const unsigned long messageInterval = 10;  //timing delay in ms between steering messages sent by node
unsigned long lastMessageTime = 0;  //keeps track of the timestamp of the last steering message sent


//node status
uint8_t errorState = 255;  //state of the node. 0: error, 1: ok, 255: offline

//steering angle values. All values are relative to min lock, and are 12 bit (0-4095), not actual angles
AMS_5600 ams5600; //instantiate 
uint16_t steeringAngle;  //for CAN message. Split into 2 bytes
uint8_t calibSteeringAngle;
uint16_t centreAngle = 2200;
#define CENTRE_ANGLE_DEVIATION_MAX 500  //maximum amount the (raw) centre angle is allowed to deviate from the ideal (2047), before the calibration function assumes something is wrong 
#define CENTRE_CAL_EEPROM_ADDRESS 0

/***CAN BUS STUFF***/
MCP2515 mcp2515(10);
struct can_frame canStatusMsg;  //status of the node
struct can_frame canMsg;  //generic CAN message for recieving data
struct can_frame calibMsg;  //cailbration CAN message
struct can_frame steeringMsg;  //main CAN message

/***FUNCTIONS***/


void sendCanMessage(){
  steeringMsg.data[0] = calibSteeringAngle; //calibrated steering angle
  steeringMsg.data[1] = steeringAngle & 0xFF; //LSB
  steeringMsg.data[2] = steeringAngle >> 8; //MSB
  mcp2515.sendMessage(&steeringMsg);
  lastMessageTime = millis();
  #ifdef DEBUG  //print steering value
  Serial.print("Raw Angle = ");
  Serial.print(steeringAngle);
  Serial.print(" | Calibrated Angle: ");
  Serial.println(calibSteeringAngle);
  #endif  //DEBUG
}

//gets the steering angle thru I2C
void readSteering(){
  steeringAngle = ams5600.getScaledAngle();  //from sensor
}


// calculates the uint8_t version of the steering angle for byte0 of the CAN Bus message
// steering angle is about 130 degrees a side
void calculateCalibratedSteering(){
  // conversion is 260 deg / 4096 bits = 0.0879

  // intermediate variable to convert angle to degrees
  int32_t angleTemp_32t = (int32_t(steeringAngle) - int32_t(centreAngle)) * 90; 
  // Serial.print("Intermediate Angle1  = ");
  // Serial.print(angleTemp_32t);
  angleTemp_32t = angleTemp_32t/1024;
  // #ifdef DEBUG  //print steering value
  // Serial.print(" | Intermediate Angle = ");
  // Serial.println(angleTemp_32t);
  // #endif  //DEBUG
  angleTemp_32t = angleTemp_32t + 127; // set 127 degrees as the centre
  //int16_t calibSteeringAngle_16t = (steeringAngle - (centreAngle - 2047))>>4;
  if(angleTemp_32t < 0){
    angleTemp_32t = 0;
  } else if (angleTemp_32t > 255){
    angleTemp_32t = 255;
  }
  calibSteeringAngle = uint8_t(angleTemp_32t);
}

//calibrates steering (either left, right or centre position)
void calibrate(uint8_t mode = 0){
  uint8_t res = mode;
  //cal min angle
  if ((mode == 0) || (mode == 1)){ 
    #ifdef DEBUG
    Serial.println("Calibrate Min. Turn steering to min end.");
    for (uint8_t i = 5; i>0;i--){ //countdown
      Serial.println(i);
      delay(1000);
    }
    Serial.println("calibrating Min");
    #else //ndef DEBUG
    delay(5000);
    #endif  //DEBUG
    uint16_t rawAngle = ams5600.setStartPosition();
    #ifdef DEBUG
    Serial.println("Min set to" + String(rawAngle));
    #endif  //DEBUG
  }
  //cal max angle
  if ((mode == 0) || (mode == 2)){ 
    #ifdef DEBUG
    Serial.println("Calibrate max. Turn steering to max end.");
    for (uint8_t i = 5; i>0;i--){ //countdown
      Serial.println(i);
      delay(1000);
    }
    Serial.println("calibrating Max");
    #else //ndef DEBUG
    delay(5000);
    #endif  //DEBUG
    uint16_t rawAngle = ams5600.setEndPosition();
    #ifdef DEBUG
    Serial.println("Max set to" + String(rawAngle));
    #endif  //DEBUG
  }
  //calibrate centre
  if (mode == 3){ 
    #ifdef DEBUG
    Serial.println("Calibrate centre. Turn steering to centre.");
    for (uint8_t i = 5; i>0;i--){ //countdown
      Serial.println(i);
      delay(1000);
    }
    Serial.println("calibrating Centre");
    #else //ndef DEBUG
    delay(5000);
    #endif  //DEBUG
    uint16_t centreAngleTemp = ams5600.getScaledAngle();

    //compare default centre point and the new centre point. If deviation is too large ( greater than CENTRE_ANGLE_DEVIATION_MAX) then reject new calibration.
    if ((int16_t(centreAngle - centreAngleTemp) > CENTRE_ANGLE_DEVIATION_MAX) || (int16_t(centreAngleTemp - centreAngle) >  CENTRE_ANGLE_DEVIATION_MAX)){ 
      res = 255;  //failed
      #ifdef DEBUG
      Serial.println("Calibration Failed! Centre point is too far away. Check if steering is centred.");
      #endif  //DEBUG
    }
    if (res != 255){  //if calibration is accepted
      centreAngle = centreAngleTemp;
      EEPROM.update(CENTRE_CAL_EEPROM_ADDRESS, centreAngle & 0xFF);
      EEPROM.update(CENTRE_CAL_EEPROM_ADDRESS+1, centreAngle >> 8);
      #ifdef DEBUG
      Serial.println("Centre set to" + String(centreAngle));
      #endif  //DEBUG
    }
  }
  //reset centre calibration
  if (mode == 4){ 
    centreAngle = 2047;
    EEPROM.update(CENTRE_CAL_EEPROM_ADDRESS, centreAngle & 0xFF);
    EEPROM.update(CENTRE_CAL_EEPROM_ADDRESS+1, centreAngle >> 8);
    #ifdef DEBUG
    Serial.println("Centre calibration reset");
    Serial.println("To" + String(centreAngle));
    #endif  //DEBUG
  }
  //send message
  calibMsg.data[0] = 0;
  calibMsg.data[1] = res;
  calibMsg.data[2] = centreAngle & 0xFF;
  calibMsg.data[3] = centreAngle >> 8;
  #ifdef DEBUG
  Serial.println("Calibration Completed: res = " + String(res) + "| Centre Angle = " + String(centreAngle));
  #endif
  mcp2515.sendMessage(&calibMsg);
}

void checkSerial(){
  if (Serial.available() > 0) {
    // read the incoming byte:
    char incomingByte = Serial.read();
    if(incomingByte == 48){
      Serial.println("Calib 0: Min-Max");
      calibrate(0);
    } else if(incomingByte == 49){
      Serial.println("Calib 1: Min");
      calibrate(1);
    } else if(incomingByte == 50){
      Serial.println("Calib 2: Max");
      calibrate(2);
    } else if(incomingByte == 51){
      Serial.println("Calib 3: Centre");
      calibrate(3);
    } else if(incomingByte == 52){
      Serial.println("Calib 4: Reset");
      calibrate(4);
    }
  }
}

void sendStatus(uint8_t status = errorState){
  errorState = status;
  #ifdef DEBUG
  Serial.print("Node status: ");
  Serial.println(errorState);
  #endif //DEBUG
  canStatusMsg.data[0] = status;
  calibMsg.data[1] = centreAngle & 0xFF;
  calibMsg.data[2] = centreAngle >> 8;
  mcp2515.sendMessage(&canStatusMsg);
}


/***SETUP***/
void setup() {
  #ifdef DEBUG  //debug mode
  Serial.begin(115200);
  Serial.println("SAS Node");
  #ifndef ARDUINO_AVR_NANO
  Serial.println("WARNING: This sketch was designed for an arduino Nano");
  #endif //#ifndef ARDUINO_AVR_NANO
  #endif //#ifdef DEBUG

  Wire.begin();

  //SET SENSOR LIMITS
  //Once the magnet is glued on we can burn the limits in maybe?
  //may need to change these limits if the magnet is shifted
  //ams5600.setStartPosition(450);
  //ams5600.setEndPosition(3160);

  //check if EEPROM has a centre calibration stored: DISABLED
  //centreAngle = (EEPROM.read(CENTRE_CAL_EEPROM_ADDRESS)) | (EEPROM.read(CENTRE_CAL_EEPROM_ADDRESS+1) << 8);

  #ifdef DEBUG  //debug mode
  Serial.print("Centre angle: ");
  Serial.println(centreAngle);
  #endif //#ifdef DEBUG

  //status message
  canStatusMsg.can_id  = SAS_STATUS_MSG_ID;
  canStatusMsg.can_dlc = 3;
  canStatusMsg.data[0] = errorState;
  canStatusMsg.data[1] = centreAngle & 0xFF;  //center low byte
  canStatusMsg.data[2] = centreAngle >> 8;  //centre high byte

  //steering angle message
  steeringMsg.can_id  = STEERING_MSG_ID;
  steeringMsg.can_dlc = 3;
  steeringMsg.data[0] = 0x00;
  steeringMsg.data[1] = 0x00;
  steeringMsg.data[1] = 0x00;

  //calibration message
  calibMsg.can_id  = CALIBRATE_STEERING_MSG_ID;
  calibMsg.can_dlc = 4;
  calibMsg.data[0] = 0x00;  //requested calibration
  calibMsg.data[1] = 0x00;  //calibration type carried out
  calibMsg.data[2] = centreAngle & 0xFF;  //center low byte
  calibMsg.data[3] = centreAngle >> 8;  //centre high byte
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  #ifdef DEBUG  //debug mode
  Serial.print("Checking if magnet sensor ok...");
  #endif //#ifdef DEBUG

  //check that magnet is detected
  unsigned long startMillis = millis();
  while(errorState != 1){
    if(ams5600.detectMagnet() == 1 ){
      #ifdef DEBUG
      Serial.print("Magnet detected. Current Magnitude: ");
      Serial.println(ams5600.getMagnitude());
      #endif //DEBUG
      sendStatus(1);
    }
    if (millis() - startMillis > 3000){ // timeout
      sendStatus(0);
      #ifdef DEBUG
      Serial.println("Can not detect magnet");
      #endif
    }
    #ifdef DEBUG
    Serial.println("Error state is not 1, this node will not boot up!");
    #endif
    delay(300);
  }
  #ifdef DEBUG
  Serial.print("Error State: ");
  Serial.println(errorState);
  #endif
}


/***MAIN LOOP :) ***/
void loop() {
  //check for calibration message
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if(canMsg.can_id == 50){ //Calibrate Steering
      uint8_t calibrationMode = canMsg.data[0];
      calibrate(calibrationMode);
    }
    if(canMsg.can_id == 7){ //Node Status Request Message ID
        sendStatus();
        //#ifdef DEBUG
        //Serial.println("Node Status Requested");
        //#endif  //DEBUG
    }
  }

  //normal operation
 
  checkSerial();  //for calibration request over serial


  if(millis() - lastMessageTime >= messageInterval){
    readSteering();
    calculateCalibratedSteering();
    sendCanMessage();
  }
 
}
