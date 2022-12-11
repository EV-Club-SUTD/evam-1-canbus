/*
FW (Front Wheel) CODE FOR EVAM

by Nigel Gomes

Generic Wheel Node Description:
FUNCTIONS:
-Read wheel speed from controller and publish to EVAM CAN Bus
-Read desired throttle value from EVAM CAN Bus and control controller
-Read other settings for the wheel (eg reverse, ECO/Boost Mode) and control controller

Designed to run on an Arduino Nano (ARDUINO_AVR_NANO)

!!THIS CODE HAS NO MILLIS() OVERFLOW PROTECTION!!

TODO:
- UNcomment lockout functionality once system is stable
- Enable boost/eco mode control(?)

Code is still under development
    Speed reading is still sometimes a bit off...

*/
#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>  //arduino-mcp2515 by autowp: https://github.com/autowp/arduino-mcp2515/
#include "FW_config.h"
#include "pulse_calculations.h"
#include "can_ids.h"


//timing

unsigned long lastRcvMillis = 0;    //time last throttle message was received
unsigned long lastSendMillis = 0;   //time last wheel speed message was sent
nodeErrorType errorState = OFFLINE;   //status of the node
unsigned long lastErrorSendMillis = 0; //time last error message was sent

//can bus stuff
struct can_frame canStatusMsg;  //status of the node
struct can_frame flWheelSpeedMsg; //front left wheel speed message
struct can_frame frWheelSpeedMsg; //front right wheel speed message
struct can_frame canMsg; //generic CAN message for recieving data
MCP2515 mcp2515(10);

//wheel speeds and throttles
motorHall lMotor;
motorHall rMotor;

bool flWheelDir = 0;    //forward = 0, reverse = 1
bool frWheelDir = 0;    //forward = 0, reverse = 1
uint16_t flWheelSpeed = 0;  //wheel speed = flWheelSpeed*0.03
uint16_t frWheelSpeed = 0;

uint8_t flThrottle = 0;
uint8_t frThrottle = 0;
bool flThrottleRev = 0;
bool frThrottleRev = 0;
uint8_t ecoBoost = 0;   //0 = normal, 1 = eco, 2 = boost
bool motorsLocked = 0;

//to update CANBus on the status of the node
void sendStatus(nodeErrorType status = errorState){
    errorState = status;
    #ifdef DEBUG
    Serial.print("Node status: ");
    Serial.println(errorState);
    #endif //DEBUG
    canStatusMsg.data[0] = status;
    mcp2515.sendMessage(&canStatusMsg);
}

void readIncomingMessages(){
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
        if(canMsg.can_id == INDIV_WHEEL_THROTTLES_MSG_ID){ //Individual Wheel Throttles
            flThrottle = canMsg.data[0];
            frThrottle = canMsg.data[1];
            if(canMsg.data[4] == 1){ //just to ensure reverse doesn't go haywire
                flThrottleRev = 1;
            } else{
                flThrottleRev = 0;
            }
            if(canMsg.data[5] == 1){ //just to ensure reverse doesn't go haywire
                frThrottleRev = 1;
            } else{
                frThrottleRev = 0;
            }
            lastRcvMillis = millis();
            #ifdef DEBUG
            //Serial.println("Throttle Data Received");
            Serial.println("Throttle: Left: " + String(flThrottle) + " | Right: " + String(frThrottle));
            #endif  //DEBUG
        }
        if(canMsg.can_id == REV_BOOST_MSG_ID){ //Boost/Eco/Reverse
            //ignore canMsg.data[0]; that is the global 'reverse' we read the 'reverse' from the individuial wheel throttle message
            ecoBoost = canMsg.data[1]; 
            #ifdef DEBUG=
            Serial.println("Boost: " + String(ecoBoost));
            #endif  //DEBUG
        }
        if(canMsg.can_id == MOTOR_LOCKOUT_MSG_ID){ //Motors Locked out
            motorsLocked = canMsg.data[0]; 
            #ifdef DEBUG
            Serial.println("Motors: " + (motorsLocked == true) ? "locked" : "unlocked");
            #endif  //DEBUG
        }
        if(canMsg.can_id == NODE_STATUS_REQUEST_MSG_ID){ //Node Status Request Message ID
            sendStatus();
            //#ifdef DEBUG
            //Serial.println("Node Status Requested");
            //#endif  //DEBUG
        }
    }
}   //readIncomingMessages

//send wheel speed messages
void sendCanMessage(){
  
    flWheelSpeedMsg.data[0] = flWheelSpeed & 0x00FF;  
    flWheelSpeedMsg.data[1] = flWheelSpeed >> 8;    
    //flWheelSpeedMsg.data[2] = flWheelDir;    
    mcp2515.sendMessage(&flWheelSpeedMsg);

    frWheelSpeedMsg.data[0] = frWheelSpeed & 0x00FF;  
    frWheelSpeedMsg.data[1] = frWheelSpeed >> 8;    
    //frWheelSpeedMsg.data[2] = frWheelDir;    
    mcp2515.sendMessage(&frWheelSpeedMsg);
    
    #ifdef DEBUG  //print brake and throttle values
    if(millis() - lastPrintMillis > PRINT_INTERVAL){
        Serial.print("Front Left Wheel Speed = ");
        Serial.print(flWheelSpeed/30);
        Serial.print(" | Front Right Wheel Speed = ");
        Serial.println(frWheelSpeed/30);
    }
   
    #endif
}

void lPulseISR(){
    lMotor.pulseMicros = micros();
    lMotor.newPulse = 1;
}

void rPulseISR(){
    rMotor.pulseMicros = micros();
    rMotor.newPulse = 1;
}

/*!
 * Toggles the Arduino pin between LOW (active) and FLOAT (inactive)
 *
 * @param   pin     The pin to toggle between float and low
 * @param   state   0 = float, 1 = low(active low)
 */
void setPinLowFLoat(uint8_t pin, bool state){
    if(state == 0){ //set to FLOAT (high-Z)
        pinMode(pin, INPUT);
    }
    else if (state == 1){   //set to LOW
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
}

void controlESCs(){
    /* TODO: Enable later
    if(motorsLocked || (errorState != 1)){  //set throttles to 0
        flThrottle = 0;
        frThrottle = 0;
    }
    */
    analogWrite(LEFT_THROTTLE_PIN, flThrottle);
    analogWrite(RIGHT_THROTTLE_PIN, frThrottle);
    setPinLowFLoat(LEFT_REVERSE_PIN, flThrottleRev);
    setPinLowFLoat(RIGHT_REVERSE_PIN, frThrottleRev);
}

void setup() {
    //status message
    canStatusMsg.can_id  = FW_STATUS_MSG_ID;
    canStatusMsg.can_dlc = 1;
    canStatusMsg.data[0] = errorState;

    //flWheel Message
    flWheelSpeedMsg.can_id  = FL_SPEED_MSG_ID;
    flWheelSpeedMsg.can_dlc = 2;
    flWheelSpeedMsg.data[0] = 0x00;
    flWheelSpeedMsg.data[1] = 0x00;
    //flWheelSpeedMsg.data[2] = 0x00;

    //frWheel Message
    frWheelSpeedMsg.can_id  = FR_SPEED_MSG_ID;
    frWheelSpeedMsg.can_dlc = 2;
    frWheelSpeedMsg.data[0] = 0x00;
    frWheelSpeedMsg.data[1] = 0x00;
    //frWheelSpeedMsg.data[2] = 0x00;

    //Pins Setup
    //Pins set to INPUT so that they are floating
    pinMode(LEFT_THROTTLE_PIN, OUTPUT);
    pinMode(RIGHT_THROTTLE_PIN, OUTPUT);
    setPinLowFLoat(LEFT_REVERSE_PIN, 0);
    setPinLowFLoat(RIGHT_REVERSE_PIN, 0);
    setPinLowFLoat(LEFT_BOOST_PIN, 0);
    setPinLowFLoat(RIGHT_BOOST_PIN, 0);
    setPinLowFLoat(LEFT_ECO_PIN, 0);
    setPinLowFLoat(RIGHT_ECO_PIN, 0);
    setPinLowFLoat(LEFT_REGEN_PIN, 0);
    setPinLowFLoat(RIGHT_REGEN_PIN, 0);

    pinMode(LEFT_PULSE_PIN, INPUT);
    pinMode(RIGHT_PULSE_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(LEFT_PULSE_PIN), lPulseISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_PULSE_PIN), rPulseISR, FALLING);

    #ifdef DEBUG  //debug mode
    Serial.begin(115200);
    Serial.println("Front Wheels");
    #ifndef ARDUINO_AVR_NANO
    Serial.print("WARNING: This sketch was designed for an arduino Nano");
    #endif //#ifndef ARDUINO_AVR_NANO
    #endif //#ifdef DEBUG
    
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
    sendStatus(OK);  
}

void loop() {
    readIncomingMessages();

    flWheelSpeed = lMotor.calculateRPM();
    frWheelSpeed = rMotor.calculateRPM();

    controlESCs();


    if(millis() - lastSendMillis >= MSG_INTERVAL){
        sendCanMessage();
        lastSendMillis = millis();
    }
    if ((errorState == OK) && (millis() - lastRcvMillis > THROTTLE_TIMEOUT)){
        sendStatus(GENERIC_ERROR);  //raise error (Throttle Timeout)
        #ifdef DEBUG
        Serial.print("Wheel Throttle Message Timeout!!");
        #endif
    }
    if ((errorState == GENERIC_ERROR) && (millis() - lastRcvMillis < THROTTLE_TIMEOUT)){
        sendStatus(OK);  //error is solved
        #ifdef DEBUG
        Serial.print("Node Working Again");
        #endif
    }

}
