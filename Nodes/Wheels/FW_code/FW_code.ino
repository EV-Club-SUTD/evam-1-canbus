/*
FW (Front Wheel) CODE FOR EVAM

Generic Wheel Node Description:
FUNCTIONS:
-Read wheel speed from controller and publish to EVAM CAN Bus
-Read desired throttle value from EVAM CAN Bus and control controller
-Read other settings for the wheel (eg reverse, ECO/Boost Mode) and control controller

Designed to run on an Arduino Nano (ARDUINO_AVR_NANO)


Code is still under development

*/
#include <SPI.h>
#include <mcp2515.h>  //arduino-mcp2515 by autowp: https://github.com/autowp/arduino-mcp2515/
#define DEBUG

//timing
#define THROTTLE_TIMEOUT 300    //timeout for not recieving throttle messages before an error is raised
#define MSG_INTERVAL 10 //interval i ms for the wheel speed messages
unsigned long lastRcvMillis = 0;    //time last throttle message was received
unsigned long lastSendMillis = 0;   //time last wheel speed message was sent
uint8_t errorState = 255;   //status of the node

//can bus stuff
struct can_frame canStatusMsg;  //status of the node
struct can_frame flWheelSpeedMsg; //front left wheel speed message
struct can_frame frWheelSpeedMsg; //front right wheel speed message
struct can_frame canMsg; //generic CAN message for recieving data
MCP2515 mcp2515(10);

//wheel speeds and throttles
bool flWheelDir = 0;    //forward = 0, reverse = 1
bool frWheelDir = 0;    //forward = 0, reverse = 1
uint16_t flWheelSpeed = 0;  //wheel speed = flWheelSpeed*0.03
uint16_t frWheelSpeed = 0;

uint8_t flThrottle = 0;
uint8_t frThrottle = 0;
bool throttleRev = 0;
uint8_t boost = 0;

//Connections to ESC
//TODO
#define LEFT_THROTTLE_PIN 1 //fill in
#define LEFT_REVERSE_PIN 2 //fill in
#define LEFT_BOOST_PIN 3 //fill in
#define RIGHT_THROTTLE_PIN 1 //fill in
#define RIGHT_REVERSE_PIN 2 //fill in
#define RIGHT_BOOST_PIN 3 //fill in


void sendCanMessage(){
  
    flWheelSpeedMsg.data[0] = flWheelSpeed & 0x00FF;  
    flWheelSpeedMsg.data[1] = flWheelSpeed >> 8;    
    flWheelSpeedMsg.data[2] = flWheelDir;    
    mcp2515.sendMessage(&flWheelSpeedMsg);

    frWheelSpeedMsg.data[0] = frWheelSpeed & 0x00FF;  
    frWheelSpeedMsg.data[1] = frWheelSpeed >> 8;    
    frWheelSpeedMsg.data[2] = frWheelDir;    
    mcp2515.sendMessage(&frWheelSpeedMsg);
    
    #ifdef DEBUG  //print brake and throttle values
    Serial.print("Front Left Wheel Speed = ");
    Serial.print(flWheelSpeed*0.03);
    Serial.print(" | Front Right Wheel Speed = ");
    Serial.print(frWheelSpeed*0.03);
    #endif
}

void readWheelSpeeds(){ //TODO: interface with hall sensor
    flWheelSpeed = 0;
    frWheelSpeed = 0;
    flWheelDir = 0;
    frWheelDir = 0;
}

void controlESCs(){
    //TODO
    analogWrite(LEFT_THROTTLE_PIN, flThrottle);
    analogWrite(RIGHT_THROTTLE_PIN, frThrottle);
    digitalWrite(LEFT_REVERSE_PIN, !throttleRev);
    digitalWrite(RIGHT_REVERSE_PIN, !throttleRev);
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
    //status message
    canStatusMsg.can_id  = 0x0D;
    canStatusMsg.can_dlc = 1;
    canStatusMsg.data[0] = errorState;

    //flWheel Message
    flWheelSpeedMsg.can_id  = 0x34;
    flWheelSpeedMsg.can_dlc = 3;
    flWheelSpeedMsg.data[0] = 0x00;
    flWheelSpeedMsg.data[1] = 0x00;
    flWheelSpeedMsg.data[2] = 0x00;

    //frWheel Message
    frWheelSpeedMsg.can_id  = 0x35;
    frWheelSpeedMsg.can_dlc = 3;
    frWheelSpeedMsg.data[0] = 0x00;
    frWheelSpeedMsg.data[1] = 0x00;
    frWheelSpeedMsg.data[2] = 0x00;

    //Pins Setup
    //TODO (add more pins as needed)
    pinMode(LEFT_THROTTLE_PIN, OUTPUT);
    pinMode(LEFT_REVERSE_PIN, OUTPUT);
    pinMode(LEFT_BOOST_PIN, OUTPUT);
    pinMode(RIGHT_THROTTLE_PIN, OUTPUT);
    pinMode(RIGHT_REVERSE_PIN, OUTPUT);
    pinMode(RIGHT_BOOST_PIN, OUTPUT);

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
    sendStatus(1);  
}

void loop() {
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
        if(canMsg.can_id == 0x30){ //Individual Wheel Throttles
            flThrottle = canMsg.data[0];
            frThrottle = canMsg.data[1];
            if(canMsg.data[4] == 0 || canMsg.data[4] == 1){ //just to ensure reverse doesn't go haywire
                throttleRev = canMsg.data[4];
            }
            else{
                throttleRev = 0;
            }
            boost = canMsg.data[5];
            lastRcvMillis = millis();
            #ifdef DEBUG
            //Serial.println("Throttle Data Received");
            Serial.println("Throttle: Left: " + String(flThrottle) + " | Right: " + String(frThrottle));
            #endif  //DEBUG

        }
    } //if message available
    readWheelSpeeds();
    controlESCs();
    if(millis() - lastSendMillis >= MSG_INTERVAL){
        sendCanMessage();
        lastSendMillis = millis();
    }
        if (millis() - lastRcvMillis > THROTTLE_TIMEOUT){
        sendStatus(0);  //raise error
    }
}