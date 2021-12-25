/*
ECU (Engine Control Unit) CODE FOR EVAM

by Nigel Gomes (https://github.com/yik3z)

FUNCTIONS:
- Read individual wheel speeds from wheel nodes and calcluate vehicle speed; publish to CAN Bus (UNTESTED)
- Read accelerator, steering (and brake) values (and other settings, like reverse, boost, etc) and calculate individual throttle values for each wheel; publish to CAN Bus
    - modify the power balance to the front&rear, and left&right wheels based on steering angle, acceleration and throttle (UNTESTED)
- Check if E stop button is pressed (100V rail); publish to CAN Bus (UNTESTED)
- Check if the battery current is too high and lower the throttle amount if it's overloaded for too long (NOT DONE YET)

Designed to run on an Arduino Nano (ARDUINO_AVR_NANO)


!This code is not millis() overflow protected!

Code is roughly working

*/

#include "Arduino.h"
#include "ECU_config.h"
#include "can_ids.h"

//timing
unsigned long lastRcvThrottleMillis = 0;    //time last throttle message was received
unsigned long lastRcvSteeringMillis = 0;    //time last steering message was received
unsigned long lastRcvWheelSpeedMillis = 0;  //time last wheel speed message was received

unsigned long lastSendWheelThrottlesMillis = 0; //time last wheel speed message was sent
unsigned long lastSendVehicleSpeedMillis = 0;   //time last vehicle speed message was sent
unsigned long lastErrorMsgMillis = 0;           //time last error message was sent
unsigned long lastEStopMsgMillis = 0;           //time last estop message was sent

unsigned long lastPrintMillis = 0;

//Errors & e-Stops
nodeErrorType errorState = OFFLINE;     //generic node status
ecuErrorType statusPart2 = NO_ERROR;        //additional information
uint8_t overCurrent = 0;                //not implemented yet
bool eStopPressed = 0;                         //whether e-Stop has been pressed. TODO: Change to initialise to 1 when the system is set up
bool motorLock = 0;                     //whether motor has been locked out through HUD. TODO: Change to initialise to 1 when the system is set up

/* Other node statuses
 * Order of bytes is as follows:
 * MSB-0;BMS;TPS;SAS;IMU;FW;RLW;RRW-LSB
 * see "ECU_config.h" for status masks
 */
uint8_t otherNodeStatuses = 0b00000000; 

/****WHEEL & SPEED DATA****/
//common
uint8_t throttle = 0;
uint8_t brakePos = 0;
int16_t steeringAngle = 128;  //0 = left lock, 255 = right lock, 127-128 = centred
bool throttleRev = 0;       //reverse. 0: forward, 1: reverse
uint8_t boostMode = 0;      //0: normal, 1: boost
uint8_t ecoMode = 0;        //0: normal, 1: eco
uint16_t vehicleSpeed = 0;  //vehicle speed. Divide by 256 to get actual speed
//bool vehicleDirection = 0;  // 0 = forward, 1 = reverse //disabled because there is no way to determine it for now.

/*  Individual wheel settings. Format:
 * [0] = front left
 * [1] = front right
 * [2] = rear left
 * [3] = rear right
 */
uint8_t wheelDirs[4];       //direction of rotation of the wheel. Forward = 0, reverse = 1
uint16_t wheelSpeeds[4];    //wheel speeds. Multiply by 0.03 to get value in RPM

/***OTHER FUNCTIONS***/
//current control
uint16_t battCurrent = 0;   //Battery Current(A) = -320 + battCurrent*0.1	
uint8_t battTemp = 0; //subtract 40 from this for the actual temperature

///abs and traction control
uint8_t wheelSpin = 0b0000;     //Set 1 for the respective bit if a wheel is spinning. order of bits: rr rl fr fl (i.e. reversed)
uint8_t wheelLockup = 0b0000;   //Set 1 for the respective bit if a wheel is locked up. order of bits: rr rl fr fl (i.e. reversed)

/****FUNCTIONS****/

/* STATUSES AND E-STOP */

//Updates CANBus on the status of the e-Stop
void sendEStopMsg(){
    #ifdef DEBUG
    Serial.print("e-Stop ");
    Serial.println(eStopPressed ? "pressed" : "released");
    #endif //DEBUG
    eStopMsg.data[0] = eStopPressed;
    mcp2515.sendMessage(&eStopMsg);
    lastEStopMsgMillis = millis();
}

//1) Checks and updates eStop variable
//2) Sends out e-stop CAN message if it has changed
//3) Updates node status
void checkEStop(){
    //may change this to an interrupt sequence later on
    if((digitalRead(E_STOP_SENSE_PIN) == LOW) && (eStopPressed == 0)){
        eStopPressed = 1;  //pressed
        sendEStopMsg();
    }
    //update that estop is released
    else if((digitalRead(E_STOP_SENSE_PIN) == HIGH) && (eStopPressed == 1)){
        eStopPressed = 0;  //released
        sendEStopMsg();
    }
}

/*!
 * Updates CANBus on the status of the node. 
 * Also updates the status of the node if provided with a status, 
 * otherwise, uses the existing status of the node
 * @param   status    new errorState (generic status) of the node
 * @param   status2   secondary information about the status of the node
 */
void sendStatus(nodeErrorType status = errorState, ecuErrorType status2 = statusPart2){
    errorState = status;
    statusPart2 = status2;
    #ifdef DEBUG
    Serial.println("Node status: " + String(errorState) + " | " + String(statusPart2));
    #endif //DEBUG
    canStatusMsg.data[0] = errorState;
    canStatusMsg.data[1] = statusPart2;
    mcp2515.sendMessage(&canStatusMsg);
    lastErrorMsgMillis = millis();
}

//reads EEPROM for saved data
//TODO Check if the EEPROM data is already saved on the board (it should be)
void readEEPROMSavedData(){
    lrPowerBalance = EEPROM.read(LR_POWER_BALANCE_ADDR);
    lrPowerScale = EEPROM.read(LR_POWER_SCALE_ADDR);
    frPowerBalance = EEPROM.read(FR_POWER_BALANCE_ADDR);
    frPowerScale = EEPROM.read(FR_POWER_SCALE_ADDR);
}

/* CAN BUS */

//Checks for and reads incoming messages
void readIncomingMessages(){
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
        if(canMsg.can_id == MOTOR_LOCKOUT_MSG_ID){ //Motor Lock
            motorLock = canMsg.data[0];
            //#ifdef DEBUG
            //Serial.print("Motors ");
            //Serial.println(motorLock ? "locked" : "released");
            //#endif  //DEBUG
        }

        if(canMsg.can_id == NODE_STATUS_REQUEST_MSG_ID){ //Node Status Request Message ID
            sendStatus();
            //#ifdef DEBUG
            //Serial.println("Node Status Requested");
            //#endif  //DEBUG
        }

        //throttle, steering, battery
        else if(canMsg.can_id == THROTTLE_BRAKE_MSG_ID){ //Throttle n Brake Position
            throttle = canMsg.data[0];
            brakePos = canMsg.data[4];
            lastRcvThrottleMillis = millis();
            if(statusPart2 == THROTTLE_TIMED_OUT){  //reset throttle timed out flag
                sendStatus(OK, NO_ERROR);
            }
            #ifdef DEBUG
            //Serial.println("Throttle: " + String(throttle) + " | Brake: " + String(brakePos));
            #endif  //DEBUG
        }
        else if(canMsg.can_id == REV_BOOST_MSG_ID){ //Reverse Selected
            throttleRev = canMsg.data[0];
            //#ifdef DEBUG
            //Serial.println("Reverse: " + String(throttleRev));
            //#endif  //DEBUG
        }
        else if(canMsg.can_id == BATT_STATS_MSG_ID){ //Battery Stats
            battCurrent = canMsg.data[2] + (canMsg.data[3]<<8);
            battTemp = canMsg.data[2];
            // #ifdef DEBUG
            // //Serial.println("Batt Current: " + String(battCurrent) + "Batt Temp: " + String(battTemp));
            // #endif  //DEBUG
        }
        else if(canMsg.can_id == STEERING_MSG_ID){ //Steering Wheel Angle
            steeringAngle = canMsg.data[0];
            lastRcvSteeringMillis = millis();
            if(statusPart2 == STEERING_TIMED_OUT){  //reset steering timed out flag
                sendStatus(errorState, NO_ERROR);
            }
            // #ifdef DEBUG
            // //Serial.println("Steering Angle: " + String(steeringAngle));
            // #endif  //DEBUG
        }

        //wheels
        else if(canMsg.can_id == FL_SPEED_MSG_ID){ //FL Wheel Speed
            wheelSpeeds[0] = canMsg.data[0] + (canMsg.data[1]<<8);
            lastRcvWheelSpeedMillis = millis();
            // #ifdef DEBUG
            // //Serial.println("FLWheel: " + String(wheelSpeeds[0]/33));   //approximation
            // #endif  //DEBUG
        }
        else if(canMsg.can_id == FR_SPEED_MSG_ID){ //FR Wheel Speed
            wheelSpeeds[1] = canMsg.data[0] + (canMsg.data[1]<<8);
            //lastRcvWheelSpeedMillis = millis();
            // #ifdef DEBUG
            // //Serial.println("FRWheel: " + String(wheelSpeeds[1]/33));   //approximation
            // #endif  //DEBUG
        }
        else if(canMsg.can_id == RL_SPEED_MSG_ID){ //RL Wheel Speed
            wheelSpeeds[2] = canMsg.data[0] + (canMsg.data[1]<<8);
            //lastRcvWheelSpeedMillis = millis();
            // #ifdef DEBUG
            // //Serial.println("RLWheel: " + String(wheelSpeeds[2]/33));   //approximation
            // #endif  //DEBUG
        }
        else if(canMsg.can_id == RR_SPEED_MSG_ID){ //RR Wheel Speed
            wheelSpeeds[3] = canMsg.data[0] + (canMsg.data[1]<<8);
            //lastRcvWheelSpeedMillis = millis();
            // #ifdef DEBUG
            // //Serial.println("RRWheel: " + String(wheelSpeeds[4]/33));   //approximation
            // #endif  //DEBUG
        }

        //differential settings
        else if(canMsg.can_id == L_R_DIFF_MSG_ID){ //Left-Right Differential Power Balance
            lrPowerBalance = canMsg.data[0];
            lrPowerScale = canMsg.data[1];
            //update EEPROM
            EEPROM.update(LR_POWER_BALANCE_ADDR, lrPowerBalance);
            EEPROM.update(LR_POWER_SCALE_ADDR, lrPowerScale);
            // #ifdef DEBUG
            // //Serial.println("L-R Diff Set to: " + String(lrDiffBalance);   
            // #endif  //DEBUG
        }

        else if(canMsg.can_id == F_R_DIFF_MSG_ID){ //Front-Rear Differential Balance
            frPowerBalance = canMsg.data[0];
            frPowerScale = canMsg.data[1];
            //update EEPROM
            EEPROM.update(FR_POWER_BALANCE_ADDR, frPowerBalance);
            EEPROM.update(FR_POWER_SCALE_ADDR, frPowerScale);
            // #ifdef DEBUG
            // //Serial.println("F-R Diff Set to: " + String(frDiffBalance);   
            // #endif  //DEBUG
        }
    } 
}   //readIncomingMessages()

void initCanBus(){
    //status message
    canStatusMsg.can_id  = ECU_STATUS_MSG_ID;
    canStatusMsg.can_dlc = 2;
    canStatusMsg.data[0] = errorState;
    canStatusMsg.data[1] = OK;

    //e-Stop Message
    eStopMsg.can_id  = E_STOP_MSG_ID;
    eStopMsg.can_dlc = 1;
    eStopMsg.data[0] = 0x00;

    //Individual wheel throttles Message
    indivWheelThrottlesMsg.can_id  = INDIV_WHEEL_THROTTLES_MSG_ID;
    indivWheelThrottlesMsg.can_dlc = 8;
    for (uint8_t i = 0; i<8; i++){
        indivWheelThrottlesMsg.data[i] = 0x00;
    }

    vehicleSpeedMsg.can_id  = VEH_SPEED_MSG_ID;
    vehicleSpeedMsg.can_dlc = 2;
    vehicleSpeedMsg.data[0] = 0x00;
    vehicleSpeedMsg.data[1] = 0x00;

    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();
}

/* CALCULATIONS */
uint16_t calcAvgRpm(uint8_t wSpin = 0b0000){
    unsigned long _rpmSum = 0;
    for (uint8_t i = 0; i < 4; i++){
        /* disabled for now
        if (((wSpin >> i) & 0b0001) == 1)  {    //filter if the wheel is spinning
          continue;
        }
        */
        _rpmSum += wheelSpeeds[i];
    }
    unsigned long _avgWheelRpm = (_rpmSum / 4);
    return uint16_t(_avgWheelRpm);
}

//compares the 4 wheel speeds to check if
//1. Any of the wheels are spinning (i.e. significantly faster than the rest)
//2. And of the wheels are locking up (significantly slower than the rest)
void checkWheelSpinLockup(){
    //TODO
    wheelSpin = 0b0000;
    wheelLockup = 0b0000;
}

/*!
 * Lowers the throttle / turns off throttle for wheels that are spinning
 */
void absTractionControl(){ 
    for (uint8_t i = 0; i < 4; i++){
        if (((wheelSpin >> i) & 0b0001) == 1)  {    //filter if the wheel is spinning
            //reduce the throttle a bit
            indivWheelThrottlesMsg.data[i] -= ((indivWheelThrottlesMsg.data[i]/10) *3); //reduce throttle for affected wheel by 30%
        }
        // if (((wheelLockup >> i) & 0b0001) == 1)  {    //filter if the wheel is locked up
        //     //can't really do anything since we can't control the brakes
        // }
    }
} 

//helper function to add the multiplier to the throttle, check for overflow, and convert to uint8_t for canbus
uint8_t addToThrottle(int16_t amtToAdd, uint8_t throttleValToAdd){
    int16_t finalThrottle = throttleValToAdd + amtToAdd;
    //check if values are out of bounds and limit it
    if(finalThrottle > 250){
        finalThrottle = 250;
    } else if(finalThrottle < 0){
        finalThrottle = 0;
    }  
    return uint8_t(finalThrottle);
}

//modifies the left vs right wheel throttles based on the steering input
void lrDifferential(){
    //TODO: Test

    //inefficient calculations, everything is long
    int16_t lrExtra = (long(throttle) * long(steeringAngle-127) * long(lrPowerBalance))/(127L*255L);

    //left side. Add the amount to the current throttle
    indivWheelThrottlesMsg.data[0] = addToThrottle(lrExtra, indivWheelThrottlesMsg.data[0]);
    indivWheelThrottlesMsg.data[2] = addToThrottle(lrExtra, indivWheelThrottlesMsg.data[2]);
    //right side. Subtract the amount from the current throttle
    indivWheelThrottlesMsg.data[1] = addToThrottle(-lrExtra, indivWheelThrottlesMsg.data[2]);
    indivWheelThrottlesMsg.data[3] = addToThrottle(-lrExtra, indivWheelThrottlesMsg.data[2]);
    
}

//modifies the front vs rear wheel throttles based on the acceleration
void frDifferential(){
    //TODO: test

    int16_t frExtra = (long(throttle) * long(throttle) * long(frPowerBalance))/(255L*255L);

    //front wheels. Subtract the amount from the current throttle
    indivWheelThrottlesMsg.data[0] = addToThrottle(-frExtra, indivWheelThrottlesMsg.data[0]);
    indivWheelThrottlesMsg.data[1] = addToThrottle(-frExtra, indivWheelThrottlesMsg.data[1]);
    //rear wheels. Add the amount to the current throttle
    indivWheelThrottlesMsg.data[1] = addToThrottle(frExtra, indivWheelThrottlesMsg.data[2]);
    indivWheelThrottlesMsg.data[3] = addToThrottle(frExtra, indivWheelThrottlesMsg.data[2]);
}

void calculateWheelThrottles(){
    //TODO test differential steering
    for(uint8_t i = 0; i<4;i++){    //set all 4 throttle values to be the same
        indivWheelThrottlesMsg.data[i] = throttle;  
    }
    for(uint8_t i = 4; i<8;i++){    //set all 4 reverse values to be the same
            indivWheelThrottlesMsg.data[i] = throttleRev;  
        }
    lrDifferential();
    frDifferential();
    checkWheelSpinLockup();
    absTractionControl();
}

void checkOverCurrent(){    //checks if battery current is too high for prolonged time
    //TODO
}

void calculateCarSpeedMsg(){
    uint16_t avgWheelRpm = 0;
    avgWheelRpm = calcAvgRpm();
    unsigned long vehicleSpeed_32 = ((avgWheelRpm/3L) * 6L * (WHEEL_CIRCUMFERENCE/10L)) *256L / (10L * 1000L);  //TODO: double check calculation when it's on the car. Also, factor of 30 is there because that's how we specified it
    vehicleSpeed = (uint16_t)vehicleSpeed_32;
}
 
void sendCarSpeedMsg(){
    vehicleSpeedMsg.data[0] = vehicleSpeed & 0xFF;
    vehicleSpeedMsg.data[1] = vehicleSpeed >> 8;
    //vehicleSpeedMsg.data[2] = vehicleDirection;   //reverse flag removed
    mcp2515.sendMessage(&vehicleSpeedMsg);
    lastSendVehicleSpeedMillis = millis();
}

//Sends CAN message for the individual wheel throttles
void sendIndivThrottlesMsg(){
    mcp2515.sendMessage(&indivWheelThrottlesMsg);
    lastSendWheelThrottlesMillis = millis();
}

#ifdef DEBUG  
//print brake and throttle values
void printThrottles(){

    Serial.print("Wheel Throttles: ");
    Serial.print(indivWheelThrottlesMsg.data[0]);
    Serial.print(" | ");
    Serial.print(indivWheelThrottlesMsg.data[1]);
    Serial.print(" | ");
    Serial.print(indivWheelThrottlesMsg.data[2]);
    Serial.print(" | ");
    Serial.println(indivWheelThrottlesMsg.data[3]);
}

//print speed value
void printSpeed(){
    Serial.print("Speed: ");
    for (uint8_t i = 0; i<4; i++){
        Serial.print(wheelSpeeds[i]);
        Serial.print(" ");
    }
    Serial.println();
}
#endif

/********* SETUP AND LOOP *************/

void setup(){
    //set up e stop pin
    pinMode(E_STOP_SENSE_PIN, INPUT);
    //attachInterrupt(digitalPinToInterrupt(E_STOP_SENSE_PIN), eStopISR, FALLING)
    #ifdef ENABLE_E_STOP_RELAY
    pinMode(E_STOP_RELAY_PIN, OUTPUT);
    #endif

    initCanBus();

    readEEPROMSavedData();

    #ifdef DEBUG  //debug mode
    Serial.begin(115200);
    Serial.println("ECU");
    #ifndef ARDUINO_AVR_NANO
    Serial.println("WARNING: This sketch was designed for an arduino Nano");
    #endif //#ifndef ARDUINO_AVR_NANO
    #endif //#ifdef DEBUG
   
    
    //check all nodes are online
    /*
    while(errorState == OFFLINE){ 
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
    */ 
      delay(1000); //hardcoded delay for now. Enable the below section later
      sendStatus(1); //ECU is ready
}

void loop(){

    readIncomingMessages();

    //checkEStop(); //TODO: ENABLE ONCE THE SYSTEM IS CONNECTED
    
    //send scheduled e-stop status message
    if(millis()-lastEStopMsgMillis > ESTOP_MSG_INTERVAL){
        sendEStopMsg();
    } 
    //send individual throttle values
    if(millis() - lastSendWheelThrottlesMillis >= WHEEL_THROTTLES_MSG_INTERVAL){
        if((eStopPressed == 0) && (motorLock == 0)){
            calculateWheelThrottles();
        } else{ //set all throttles to 0
            indivWheelThrottlesMsg.data[0] = 0x00;
            indivWheelThrottlesMsg.data[1] = 0x00;
            indivWheelThrottlesMsg.data[2] = 0x00;
            indivWheelThrottlesMsg.data[3] = 0x00;
        }
        sendIndivThrottlesMsg();
        #ifdef DEBUG
        //Serial.println("Throttle In: " + String(throttle));
        #endif
    }
    //send vehicle speed
    if(millis() - lastSendVehicleSpeedMillis >= VEHICLE_SPEED_MSG_INTERVAL){
        calculateCarSpeedMsg();
        sendCarSpeedMsg();
    }

    //timeouts
    if (millis() - lastErrorMsgMillis> ERROR_MSG_INTERVAL){
        if (millis() - lastRcvThrottleMillis > THROTTLE_TIMEOUT){
        sendStatus(GENERIC_ERROR, THROTTLE_TIMED_OUT);  //raise error
        }
        if (millis() - lastRcvSteeringMillis > THROTTLE_TIMEOUT){
        sendStatus(errorState, STEERING_TIMED_OUT);
        }
        if (millis() - lastRcvWheelSpeedMillis > WHEEL_SPEED_TIMEOUT){
        sendStatus(errorState, WHEEL_SPEED_TIMED_OUT);
        }
 
    }
    if(millis() - lastPrintMillis > PRINT_INTERVAL){
        printThrottles();
        printSpeed();
    }
}
