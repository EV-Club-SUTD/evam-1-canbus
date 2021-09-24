/*
  Analog Input

  Demonstrates analog input by reading an analog sensor on analog pin 0 and
  turning on and off a light emitting diode(LED) connected to digital pin 13.
  The amount of time the LED will be on and off depends on the value obtained
  by analogRead().

  The circuit:
  - potentiometer
    center pin of the potentiometer to the analog input 0
    one side pin (either one) to ground
    the other side pin to +5V
  - LED
    anode (long leg) attached to digital output 13
    cathode (short leg) attached to ground

  - Note: because most Arduinos have a built-in LED attached to pin 13 on the
    board, the LED is optional.

  created by David Cuartielles
  modified 30 Aug 2011
  By Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogInput
*/
#include <EwmaT.h>

const uint8_t sensorPin = A0;    // select the input pin for the potentiometer
//uint16_t sensorValue = 0;  // variable to store the value coming from the sensor
EwmaT <uint32_t> filterTest(3, 100);
unsigned long lastMillis = 0;


void setup() {
  Serial.begin(115200);
}

void loop() {
  // read the value from the sensor:
  uint16_t sensorValue = analogRead(sensorPin)<<4;
  uint32_t filteredValue = filterTest.filter(sensorValue);
  if (millis() - lastMillis > 100){
    uint8_t val8 = filteredValue>>6;
    uint8_t valInt = val8/20;
    uint8_t valDec = (val8%20)*5;
    //NO USING FLOATS!!
    Serial.println("5V = " + String(valInt) + ((valDec<10) ? (".0") : (".")) + String(valDec));
    lastMillis = millis();
  }
}
