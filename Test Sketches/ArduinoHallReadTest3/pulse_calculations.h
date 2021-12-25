#ifndef _PULSE_CALC_H_
#define _PULSE_CALC_H_

#include <Arduino.h>

/*
 * Library to read hall pulses and calculate wheel RPM
 * Developed for EVAM motor controller (APT AE96600)
 * And designed to run on Arduino
 * Usage: 
 * 1) decalre an ISR in your main programme,
 * 2) make an instance of motorHall in the main pprogramme
 * 3) make an ISR (interrupt service routine) as follows:
 * void pulseISR(){
 *    hallMotorInstance.pulseMicros = micros();
 *    hallMotorInstance.newPulse = 1;
 * }
 * CODE IS STILL UNDER DEVELOPMENT AND UNTESTED
 */

#define WHEEL_STOP_TIMEOUT 900000
#define MIN_HALL_INTERVAL_MICROS 1800 //anything below this is cut off as erroneous data. Set as such to respect the uint16_t limit for the RPM

class motorHall {
    public:
        uint16_t motorSpeed = 0;    //is actually 30 times the real RPM. Increased for more resolution
        volatile uint8_t newPulse = 0; 
        unsigned long pulseMicros = 0;
        //unsigned long printMillis = 0;

    public:

        uint16_t calculateRPM();   //the main method to run. It will run the other 2

    private:
        void calculateInterval();
        void calculateRPMfromInterval();
        unsigned long lastPulseMicros = 0;
        unsigned long pulseInterval = 1000; //Time between 2 Hall pulses. Starting value is non-zero so there's no divide by zero error
};




#endif //_PULSE_CALC_H_
