#pragma once

// These define's must be placed at the beginning before #include "ESP32_PWM.h"
// _PWM_LOGLEVEL_ from 0 to 4
// Don't define _PWM_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define _PWM_LOGLEVEL_      0

#define USING_MICROS_RESOLUTION       true    //false 

#include <math.h>
#include "HardwareTimer.h"
#include <HardwareSerial.h>
#include "Arduino.h"

#define NUM_OF_PINS   3

#define NUMBER_ISR_PWMS         NUM_OF_PINS

/////////////////////////////////////////////////////

extern uint32_t freq;
extern HardwareSerial Serial3;

//////////////////////////////////////////////////////


class MOTOR_PWM {
public:
    MOTOR_PWM(uint32_t phaseAPin, uint32_t phaseBpin, uint32_t phaseCpin);
    void setup();
    void move(float electricalAngle);
    void updateDutyCycle(uint32_t motorIndex);

private:
    uint32_t pins[NUM_OF_PINS] = {0, 0, 0}; // motor0 pins
    uint32_t channels[NUM_OF_PINS] = {0, 0, 0};
    uint32_t dutyCycles[NUM_OF_PINS] = { 65535, 0, 0 }; // Initial duty cycles
    HardwareTimer* timers[NUM_OF_PINS];
    static constexpr uint32_t maximumPWMDutyCycle = 65535; // Adjust this value based on your PWM resolution
};
