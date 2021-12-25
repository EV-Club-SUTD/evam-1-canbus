/*
RL (REAR LIGHTS) CODE FOR EVAM

by Nigel Gomes (https://github.com/yik3z)

FUNCTIONS:
-Light up LED based on messages send on CAN Bus (not yet implemented)
--Turn on indicator (not yet implemented)
--Turn on brake lights when braking(not yet implemented)
--Fade between lighting values if desired (not yet implemented)


Designed to run on an Arduino Nano (ARDUINO_AVR_NANO)
and use WS2815 (or WS2812B) LED strips

Using the FASTLED library

!This code is not millis() overflow protected!
*/

//DEVELOPMENT IN PROGRESS//

#include "Arduino.h"
#include <SPI.h>
#include <mcp2515.h>  //arduino-mcp2515 by autowp: https://github.com/autowp/arduino-mcp2515/
#include <FastLED.h>

#define DEBUG


//timing stuff
#define PRINT_INTERVAL 1000
unsigned long lastPrintTime = 0;

#define LED_UPDATE_INTERVAL 30
unsigned long lastLEDUpdateMillis = 0;


//node status
uint8_t errorState = 255;  //state of the node. 0: error, 1: ok, 255: offline

/* FASTLED (LED) SETUP */
#define LED1_PIN 2  //data pin of the first LED strip
#define LED2_PIN 3  //data pin of the first LED strip
#define LED3_PIN 4  //data pin of the first LED strip

#define NUM_LEDS 38 //per strip
#define HALF_NUM_LEDS 19 //

CRGB strip1[NUM_LEDS];
CRGB strip2[NUM_LEDS];
CRGB strip3[NUM_LEDS];

//variables to track stuff
uint8_t r = 0;
uint8_t g = 0;
uint8_t b = 0;
uint8_t brakeVal = 0;         //amount brake is pressed in percent (0-100%)
uint8_t indicator = 0;        //0 = none, 1 = left, 2 = right

/* CAN BUS STUFF */
MCP2515 mcp2515(10);
struct can_frame canStatusMsg;  //status of the node
struct can_frame canMsg;        //generic CAN message for recieving data

#ifdef DEBUG  
void printLEDData(uint8_t ledNumber){
  Serial.print("LED ");
  Serial.print(ledNumber);
  Serial.print(": ");
  Serial.print(r);
  Serial.print("| ");
  Serial.print(g);
  Serial.print("| ");
  Serial.println(b);
}

void debugSetLEDColour(){
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    strip1[i].r = 128;
    strip1[i].g = 128;
    strip1[i].b = 128;
  }
}

#endif 

//Overlays part of the rear lights with red (no matter what colour the rear lights are set to).
//The amount of the strip that is overlaid with red depends on how much the brake pedal is pressed
void overrideBrake(){

  //assuming NUM_LEDs is going to stay as 38, it means we have 19 LEDs a side
  //so each LED can represent 5%
  uint8_t ledsToLightUp = (brakeVal / 5);
  if(ledsToLightUp>HALF_NUM_LEDS){ //assuming leds to light up is an even number
    ledsToLightUp = HALF_NUM_LEDS;
  }
  //set right half
  for (uint8_t i = HALF_NUM_LEDS; i < (HALF_NUM_LEDS+ledsToLightUp); i++) {
    strip1[i].r = 255;
    strip1[i].g = 0;
    strip1[i].b = 0;
  }
  //set left half
  for (int8_t i = (HALF_NUM_LEDS-1); i >= (HALF_NUM_LEDS-ledsToLightUp); i--) {
    strip1[i].r = 255;
    strip1[i].g = 0;
    strip1[i].b = 0;
  }
}

void readIncomingMessages(){
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if(canMsg.can_id == 97){ //Rear Light Value; update the LED colour
      //update the global colour variables
      r = canMsg.data[0];
      g = canMsg.data[1];
      b = canMsg.data[2];
      #ifdef DEBUG
      Serial.println("Lighting Message Received");
      printLEDData(1);
      #endif
    } //can_id == 96
    //for brakes only
    if(canMsg.can_id == 32){ //(throttle and) brake position
      uint8_t brakeValRaw = canMsg.data[4];
      uint16_t brakeVal_16 = (brakeValRaw * 4) / 10;
      brakeVal = uint8_t(brakeVal_16);
      // #ifdef DEBUG
      // Serial.println("Brake Message Received");
      // #endif
    } //can_id == 32
    if(canMsg.can_id == 7){ //Node Status Request Message ID
      sendStatus();
      //#ifdef DEBUG
      //Serial.println("Node Status Requested");
      //#endif  //DEBUG
    }
  } //can message received
} //readIncomingMessages()



void updateLEDs(){
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    strip1[i].r = r;
    strip1[i].g = g;
    strip1[i].b = b;
  }
  overrideBrake();
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
  Serial.println("Rear LED Node");
  #ifndef ARDUINO_AVR_NANO
  Serial.println("WARNING: This sketch was designed for an arduino Nano");
  #endif //#ifndef ARDUINO_AVR_NANO
  #endif //#ifdef DEBUG

  //fastLED
  FastLED.addLeds<WS2813, LED1_PIN>(strip1, NUM_LEDS);   //check RGB order. Check if WS2813 can be used as a replacement for WS2815
  FastLED.addLeds<WS2813, LED2_PIN>(strip2, NUM_LEDS);
  FastLED.addLeds<WS2813, LED3_PIN>(strip3, NUM_LEDS);
  FastLED.setCorrection(TypicalLEDStrip);                //check


  //status message
  canStatusMsg.can_id  = 17;  //or 18 or 19, depending on which node
  canStatusMsg.can_dlc = 1;
  canStatusMsg.data[0] = errorState;


  //initialise CAN Bus module
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  sendStatus(1);  
}

void loop(){
  readIncomingMessages();
  updateLEDs();
  #ifdef DEBUG  
  //print LED colour
  if(millis() - lastPrintTime >= PRINT_INTERVAL){
    //debugSetLEDColour();  //for testing
    printLEDData(1);
    lastPrintTime = millis();
  }
  #endif 
  //delay to limit framerate
  if(millis() - lastLEDUpdateMillis >= LED_UPDATE_INTERVAL){
    FastLED.show();
    lastLEDUpdateMillis = millis();
  }
}
