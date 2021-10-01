/*
ECU (Engine Control Unit) CODE FOR EVAM

FUNCTIONS:
- Read individual wheel speeds from wheel nodes and calcluate vehicle speed; publish to CAN Bus
- Read accelerator, steering (and brake) values (and other settings, like reverse, boost, etc) and calculate individual throttle values for each wheel; publish to CAN Bus
- Check if E stop button is pressed (100V rail); publish to CAN Bus
- Check if the battery current is too high and lower the throttle amount if it's overloaded for too long

Designed to run on an Arduino Nano (ARDUINO_AVR_NANO)

Code is still under development

*/
#include <SPI.h>
#include <mcp2515.h>  //arduino-mcp2515 by autowp: https://github.com/autowp/arduino-mcp2515/
#include "ECU_config.h"

//timing
unsigned long lastRcvThrottleMillis = 0;    //time last throttle message was received
unsigned long lastRcvSteeringMillis = 0;    //time last steering message was received
unsigned long lastRcvWheelSpeedMillis = 0;  //time last wheel speed message was received

unsigned long lastSendWheelThrottlesMillis = 0; //time last wheel speed message was sent
unsigned long lastSendVehicleSpeedMillis = 0;   //time last vehicle speed message was sent

//Errors
uint8_t errorState = 255;   //status of the node
uint8_t overCurrent = 0;
uint8_t eStop = 0;
//uint8_t eStopPressed = 0; //flag for eStop interrupt

/* Other node statuses
 * Order of bytes is as follows:
 * MSB-0;BMS;TPS;SAS;IMU;FW;RLW;RRW-LSB
 * see "ECU_config.h" for status masks
 */
uint8_t otherNodeStatuses = 0b00000000; 

//can bus
MCP2515 mcp2515(10);
struct can_frame canMsg; //generic CAN message for recieving data
struct can_frame canStatusMsg;  //status of the node
struct can_frame eStopMsg; //e-Stop message
struct can_frame indivWheelThrottlesMsg; // individual wheel throttles
struct can_frame vehicleSpeedMsg; // speed message

/****WHEEL & SPEED DATA****/
//common
uint8_t throttle = 0;
uint8_t brakePos = 0;
int16_t steeringAngle = 0; //divide by 10 to get actual steering angle
bool throttleRev = 0;   //reverse. 0: forward, 1: reverse
uint8_t boostMode = 0;  //0: normal, 1: eco, 2: boost
uint16_t vehicleSpeed = 0;  //vehicle speed. Divide by 256 to get actual speed

/*  Individual wheel settings. Format:
 * [0] = front left
 * [1] = front right
 * [2] = rear left
 * [3] = rear right
 */
uint8_t wheelDirs[4];       //direction of rotation of the wheel. Forward = 0, reverse = 1
uint16_t wheelSpeeds[4];    //wheel speeds. Multiply by 0.03 to get value in RPM
uint8_t wheelThrottles[4];  // wheel throttle values to send to nodes

/***OTHER FUNCTIONS***/
//current control
uint16_t battCurrent = 0;   //Battery Current(A) = -320 + battCurrent*0.1	
uint8_t battTemp = 0; //subtract 40 from this for the actual temperature

///abs and traction control
uint8_t wheelSpin = 0b0000;     //Set 1 for the respective bit if a wheel is spinning. order of bits: rr rl fr fl (i.e. reversed)
uint8_t lrDiffBalance = 0;      //Left-Right Differential Power Balance. 0: equal distribution between inner and outer wheel, 255: all power to outer wheel
uint8_t lrDiffScale = 1;        //working it out
uint8_t frDiffBalance = 127;    //Front-Rear Differential Balance. 0: 100%front, 127: 50-50, 255: 100% rear
uint8_t frDiffScale = 1;        //working it out

/****FUNCTIONS****/
/*
void eStopISR(){
    eStopPressed = 1;
}
*/

uint16_t calcAvgRpm(uint8_t wSpin = 0b0000){
    uint32_t rpmSum = 0;
    for (uint8_t i = 0; i < 4; i++){
        if (((wSpin >> i) & 0b0001) == 1)  {    //filter if the wheel is spinning
          continue;
        }
        rpmSum += wheelSpeeds[i];
    }
    uint16_t avgWheelRpm = (uint16_t)(rpmSum / 4);
    return avgWheelRpm;
}

void absTractionControl(){ //calculates if any of the wheel speeds are a lot more than the rest and lowers the throttle for it significantly/turns off throttle for it
    //TODO
    uint16_t avgWheelRpm = calcAvgRpm(wheelSpin);
    //idk what to do 
    
} 

void calculateWheelThrottles(){
    //TODO use throttle and steering, differential balance
    for(uint8_t i = 0; i<4;i++){
            wheelThrottles[i] = throttle;
        }
    absTractionControl();
    if(eStop){
        for(int i = 0; i<4;i++){
            wheelThrottles[i] = 0;
        }
    }
}

void checkOverCurrent(){    //checks if battery current is too high for prolonged time
    //TODO
}

void calculateSendCarSpeedMsg(){
    //check if wheel directions are all the same. Is there a better way to do this??
    bool wheelsSameDir = true;
    uint16_t avgWheelRpm = 0;
    if (wheelsSameDir){
        avgWheelRpm = calcAvgRpm();
    }
    else{
        avgWheelRpm = wheelSpeeds[0];
    }

    //idk is this the right way to do things??
    uint32_t vehicleSpeed32bit = avgWheelRpm * 60 * WHEEL_CIRCUMFERENCE * 256 / (1000 * 100 * 33);    
    vehicleSpeed = (uint16_t)vehicleSpeed32bit;

    vehicleSpeedMsg.data[0] = vehicleSpeed & 0xFF;
    vehicleSpeedMsg.data[1] = vehicleSpeed >> 8;
    //TODO: reverse or forward
}

//to update CANBus on the status of the node
void sendStatus(uint8_t status = errorState){
    errorState = status;
    #ifdef DEBUG
    Serial.print("Node status: ");
    Serial.println(errorState);
    #endif //DEBUG
    canStatusMsg.data[0] = status;
    mcp2515.sendMessage(&canStatusMsg);
}

//to update CANBus on the status of the node
void sendEStopMsg(){
    #ifdef DEBUG
    Serial.print("e-Stop ");
    Serial.println(eStop ? "pressed" : "released");
    #endif //DEBUG
    eStopMsg.data[0] = eStop;
    mcp2515.sendMessage(&eStopMsg);
}

void sendIndivThrottlesMsg(){
    for(int i = 0; i <4; i++){
        indivWheelThrottlesMsg.data[i] = wheelThrottles[i];  
    }
    mcp2515.sendMessage(&indivWheelThrottlesMsg);

    #ifdef DEBUG  //print brake and throttle values
    Serial.print("Wheel Throttles: ");
    Serial.print(wheelThrottles[0]/33);
    Serial.print(" | ");
    Serial.print(wheelThrottles[1]/33);
    Serial.print(" | ");
    Serial.print(wheelThrottles[2]/33);
    Serial.print(" | ");
    Serial.print(wheelThrottles[3]/33);
    #endif
}


void setup(){
    //set up e stop pin
    pinMode(E_STOP_SENSE_PIN, INPUT);
    //attachInterrupt(digitalPinToInterrupt(E_STOP_SENSE_PIN), eStopISR, FALLING)
    #ifdef ENABLE_E_STOP_RELAY
    pinMode(E_STOP_RELAY_PIN, OUTPUT);
    #endif

    /***initialise can bus messages***/

    //status message
    canStatusMsg.can_id  = 0x08;
    canStatusMsg.can_dlc = 1;
    canStatusMsg.data[0] = errorState;

    //e-Stop Message
    eStopMsg.can_id  = 0x04;
    eStopMsg.can_dlc = 1;
    eStopMsg.data[0] = 0x00;

    //Individual wheel throttles Message
    indivWheelThrottlesMsg.can_id  = 0x30;
    indivWheelThrottlesMsg.can_dlc = 4;
    indivWheelThrottlesMsg.data[0] = 0x00;
    indivWheelThrottlesMsg.data[1] = 0x00;
    indivWheelThrottlesMsg.data[2] = 0x00;
    indivWheelThrottlesMsg.data[3] = 0x00;

    vehicleSpeedMsg.can_id  = 0x37;
    vehicleSpeedMsg.can_dlc = 3;
    vehicleSpeedMsg.data[0] = 0x00;
    vehicleSpeedMsg.data[1] = 0x00;
    vehicleSpeedMsg.data[2] = 0x00;

    #ifdef DEBUG  //debug mode
    Serial.begin(115200);
    Serial.println("ECU");
    #ifndef ARDUINO_AVR_NANO
    Serial.print("WARNING: This sketch was designed for an arduino Nano");
    #endif //#ifndef ARDUINO_AVR_NANO
    #endif //#ifdef DEBUG
    
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();

    /* MASK TESTING
     * MCP2515::ERROR setFilterMask(MCP2515::MASK0, false, 0b??)
     * MCP2515::ERROR setFilter(MCP2515::RXF0, false, const uint32_t ulData)
     */

    //check all nodes are online
    while(errorState == 255){ 
        if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
            if(canMsg.can_id == 0x09){ //BMS
                if(canMsg.data[0] == 1){
                    otherNodeStatuses = (otherNodeStatuses | BMS_STATUS_MASK);
                    #ifdef DEBUG
                    Serial.println("BMS Online");
                    #endif //DEBUG
                }
            }
            else if(canMsg.can_id == 0x0A){ //TPS
                if(canMsg.data[0] == 1){
                    otherNodeStatuses = (otherNodeStatuses | TPS_STATUS_MASK);
                    #ifdef DEBUG
                    Serial.println("TPS Online");
                    #endif //DEBUG
                }
            }
            else if(canMsg.can_id == 0x0B){ //SAS
                if(canMsg.data[0] == 1){
                    otherNodeStatuses = (otherNodeStatuses | SAS_STATUS_MASK);
                    #ifdef DEBUG
                    Serial.println("SAS Online");
                    #endif //DEBUG
                }
            }
            else if(canMsg.can_id == 0x0D){ //FW
                if(canMsg.data[0] == 1){
                    otherNodeStatuses = (otherNodeStatuses | FW_STATUS_MASK);
                    #ifdef DEBUG
                    Serial.println("FW Online");
                    #endif //DEBUG
                }
            }
            else if(canMsg.can_id == 0x0F){ //RLW
                if(canMsg.data[0] == 1){
                    otherNodeStatuses = (otherNodeStatuses | RLW_STATUS_MASK);
                    #ifdef DEBUG
                    Serial.println("RLW Online");
                    #endif //DEBUG
                }
            }
            else if(canMsg.can_id == 0x10){ //RRW
                if(canMsg.data[0] == 1){
                    otherNodeStatuses = (otherNodeStatuses | RRW_STATUS_MASK);
                    #ifdef DEBUG
                    Serial.println("RRW Online");
                    #endif //DEBUG
                }
            }
        }
        if((otherNodeStatuses & 0b11100111) == 0b01100111){  //filter out SAS and IMU (position 3 and 4)
        sendStatus(1); //ECU is ready
        }
    } 
}

void loop(){   //TODO
    /* check and update eStop */
    //update that eStop is pressed
    if((digitalRead(E_STOP_SENSE_PIN) == HIGH) && (eStop == 0)){
        eStop = 1;
        sendEStopMsg();
        sendStatus(2);
    }
    //update that estop is released
    else if((digitalRead(E_STOP_SENSE_PIN) == LOW) && (eStop == 1)){
        eStop = 0;
        sendEStopMsg();
        sendStatus(0);
    }

    /***read incoming messages***/
    //throttle, steering, battery
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
        if(canMsg.can_id == 0x20){ //Throttle n Brake Position
            throttle = canMsg.data[0];
            brakePos = canMsg.data[4];
            lastRcvThrottleMillis = millis();
            #ifdef DEBUG
            //Serial.println("Throttle: " + String(throttle));
            #endif  //DEBUG
        }
        else if(canMsg.can_id == 0x24){ //Battery Stats
            battCurrent = canMsg.data[2] + (canMsg.data[3]<<8);
            battTemp = canMsg.data[2];
            // #ifdef DEBUG
            // //Serial.println();
            // #endif  //DEBUG
        }
        else if(canMsg.can_id == 0x2C){ //Steering Wheel Angle
            steeringAngle = canMsg.data[0] + (canMsg.data[1]<<8) - 1800;
            lastRcvSteeringMillis = millis();
            // #ifdef DEBUG
            // //Serial.println("Steering Angle: " + String(steeringAngle/10));
            // #endif  //DEBUG
        }

        //wheels
        else if(canMsg.can_id == 0x34){ //FL Wheel Speed
            wheelSpeeds[0] = canMsg.data[0] + (canMsg.data[1]<<8);
            lastRcvWheelSpeedMillis = millis();
            // #ifdef DEBUG
            // //Serial.println("FLWheel: " + String(wheelSpeeds[0]/33));   //approximation
            // #endif  //DEBUG
        }
        else if(canMsg.can_id == 0x35){ //FR Wheel Speed
            wheelSpeeds[1] = canMsg.data[0] + (canMsg.data[1]<<8);
            //lastRcvWheelSpeedMillis = millis();
            // #ifdef DEBUG
            // //Serial.println("FRWheel: " + String(wheelSpeeds[1]/33));   //approximation
            // #endif  //DEBUG
        }
        else if(canMsg.can_id == 0x36){ //RL Wheel Speed
            wheelSpeeds[2] = canMsg.data[0] + (canMsg.data[1]<<8);
            //lastRcvWheelSpeedMillis = millis();
            // #ifdef DEBUG
            // //Serial.println("RLWheel: " + String(wheelSpeeds[2]/33));   //approximation
            // #endif  //DEBUG
        }
        else if(canMsg.can_id == 0x37){ //RR Wheel Speed
            wheelSpeeds[3] = canMsg.data[0] + (canMsg.data[1]<<8);
            //lastRcvWheelSpeedMillis = millis();
            // #ifdef DEBUG
            // //Serial.println("RRWheel: " + String(wheelSpeeds[4]/33));   //approximation
            // #endif  //DEBUG
        }

        //differential settings
        else if(canMsg.can_id == 0x3C){ //Left-Right Differential Power Balance
            lrDiffBalance = canMsg.data[0];
            lrDiffScale = canMsg.data[1];
            // #ifdef DEBUG
            // //Serial.println("L-R Diff Set to: " + String(lrDiffBalance);   
            // #endif  //DEBUG
        }

        else if(canMsg.can_id == 0x3D){ //Front-Rear Differential Balance
            frDiffBalance = canMsg.data[0];
            frDiffScale = canMsg.data[1];
            // #ifdef DEBUG
            // //Serial.println("F-R Diff Set to: " + String(frDiffBalance);   
            // #endif  //DEBUG
        }
    } //read incoming messages

    calculateWheelThrottles();

    //send individual throttle values
    if(millis() - lastSendWheelThrottlesMillis >= WHEEL_THROTTLES_MSG_INTERVAL){
        sendIndivThrottlesMsg();
        lastSendWheelThrottlesMillis = millis();
    }
    //send vehicle speed
    if(millis() - lastSendVehicleSpeedMillis >= VEHICLE_SPEED_MSG_INTERVAL){
        calculateSendCarSpeedMsg();
        lastSendVehicleSpeedMillis = millis();
    }

    //timeouts
    if (millis() - lastRcvThrottleMillis > THROTTLE_TIMEOUT){
    sendStatus(0);  //raise error
    }
    if (millis() - lastRcvSteeringMillis > THROTTLE_TIMEOUT){
    sendStatus(0);  //raise error
    }
    if (millis() - lastRcvWheelSpeedMillis > WHEEL_SPEED_TIMEOUT){
    sendStatus(0);  //raise error
    }
}
