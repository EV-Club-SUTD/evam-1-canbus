#include "pulse_calculations.h"

#define REVERSE_PIN 4
#define BOOST_PIN 1
#define ECO_PIN 2
#define ACCELERATOR_PIN A0
#define THROTTLE_PIN 5
#define THROTTLE_PIN_2 6
#define LEFT_PULSE_PIN 2
#define RIGHT_PULSE_PIN 3

uint8_t throttle = 0;

motorHall lMotor;
motorHall rMotor;

void setPinLowFLoat(uint8_t pin, bool state){
    if(state == 0){ //set to FLOAT (high-Z)
        pinMode(pin, INPUT);
    }
    else if (state == 1){   //set to LOW
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
}

void lPulseISR(){
    lMotor.pulseMicros = micros();
    lMotor.newPulse = 1;
}

void rPulseISR(){
    rMotor.pulseMicros = micros();
    rMotor.newPulse = 1;
}

void setup() {
  // put your setup code here, to run once:
    setPinLowFLoat(REVERSE_PIN, 0);
    setPinLowFLoat(ECO_PIN, 0);
    setPinLowFLoat(BOOST_PIN, 0);
    pinMode(ACCELERATOR_PIN, INPUT);
    pinMode(THROTTLE_PIN, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(LEFT_PULSE_PIN), lPulseISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_PULSE_PIN), rPulseISR, FALLING);

     Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
    uint16_t throttle_16 = (ACCELERATOR_PIN);
    throttle = throttle_16 >> 2;
    //Serial.println(throttle);
    analogWrite(THROTTLE_PIN, throttle);
    uint16_t flWheelSpeed = lMotor.calculateRPM();
    uint16_t frWheelSpeed = rMotor.calculateRPM();
    Serial.print(flWheelSpeed/30);
    Serial.print(" | ");
    Serial.println(frWheelSpeed/30);
}
