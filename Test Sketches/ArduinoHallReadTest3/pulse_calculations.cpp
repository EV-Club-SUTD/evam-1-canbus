#include "pulse_calculations.h"
#include "Arduino.h"


void motorHall::calculateInterval(){
    noInterrupts();

    unsigned long _pulseInterval = pulseMicros - lastPulseMicros;
    lastPulseMicros = pulseMicros;
    if (_pulseInterval > MIN_HALL_INTERVAL_MICROS){ //check that the interval isn't out of bounds
        pulseInterval = _pulseInterval;
    }

    interrupts();
}

void motorHall::calculateRPMfromInterval(){
    motorSpeed = 60 * 1000000 * 30 / (pulseInterval * 16);    //attempted integer calculation
    //float motorSpeedNew = 60.0 * 1000000.0 * 30.0 / (pulseInterval * 16.0);    //can we change float calculations to integer??
    //motorSpeed = uint16_t(motorSpeedNew); //nofilter
    //motorSpeed = (motorSpeed *3 + uint16_t(motorSpeedNew) * 7)/10;  //exponential filter
}

uint16_t motorHall::calculateRPM(){
    if(newPulse == 1){
        calculateInterval();
        newPulse = 0;
        calculateRPMfromInterval();
    }
    if (micros() - lastPulseMicros > WHEEL_STOP_TIMEOUT){
        motorSpeed = 0;
    }
    return motorSpeed;
}
