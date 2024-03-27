#pragma once

#define _PWM_LOGLEVEL_        0

#if ( defined(ARDUINO_NANO_RP2040_CONNECT) || defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ADAFRUIT_FEATHER_RP2040) || \
      defined(ARDUINO_GENERIC_RP2040) ) && defined(ARDUINO_ARCH_MBED)

#if(_PWM_LOGLEVEL_>3)
  #warning USING_MBED_RP2040_PWM
#endif

#elif ( defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ADAFRUIT_FEATHER_RP2040) || \
        defined(ARDUINO_GENERIC_RP2040) ) && !defined(ARDUINO_ARCH_MBED)

#if(_PWM_LOGLEVEL_>3)
  #warning USING_RP2040_PWM
#endif
#else
#error This code is intended to run on the RP2040 mbed_nano, mbed_rp2040 or arduino-pico platform! Please check your Tools->Board setting.
#endif

#include "Arduino.h"
#include "RP2040_PWM.h"
#include "svpwm.h"
#include "svpwm.c"
#include "adc.h"

#define NUM_OF_PINS   3

#define NUMBER_ISR_PWMS         NUM_OF_PINS

/////////////////////////////////////////////////////

class MOTOR_PWM {
public:
    MOTOR_PWM(uint32_t en, uint32_t aPWM, uint32_t bPWM, uint32_t cPWM, uint32_t sense1, uint32_t sense2, int index1, int index2);
    void setup();
    void move(float electricalAngle, bool cw);
    int phase = 0; // 0 = A high, 1 = B high, 2 = C high
    float ratioASense1 = 0.0f;
    float ratioBSense1 = 0.0f;
    // Current Sensing
    void updateCurrents();
    void updateReadings();
    float ia = 0.0f;
    float ib = 0.0f;
    float ic = 0.0f;
private:
    void updateDutyCycles();
    uint32_t pwmPins[NUM_OF_PINS]; // phase pwm pins
    uint32_t enPin; // phase enable pins`
    tSVPWM sSVPWM = SVPWM_DEFAULTS;
    float initialDutyCycles[NUM_OF_PINS] = {75.0, 0.0, 0.0}; // Initial duty cycles
    RP2040_PWM* PWM_Instance[NUM_OF_PINS];
    float freq = 100000.0f;
    float maximumPWMDutyCycle = 100.0f; // Adjust this value based on your PWM resolution
    float volatageMagintude = 4.0f; // this will be set dynamically in future
    float normalizedSenseThreshold = 0.5 * volatageMagintude / 7.0f;
    // Current Sensing
    // float adc_to_current(uint16_t, uint16_t);
    uint16_t adc_1_offset = 0;
    uint16_t adc_2_offset = 0;
    uint16_t adc_1_max = 485;
    uint16_t adc_2_max = 435;
    uint32_t sense1Pin;
    uint32_t sense2Pin;
    int index_1;
    int index_2;
    float emaSense1 = 0;
    float emaSense2 = 0;
    const float alpha = 0.1; // Example smoothing factor

    void updateEMASense1(uint16_t currentReading) {
      emaSense1 = alpha * (currentReading) + (1 - alpha) * emaSense1;
    }

    void updateEMASense2(uint16_t currentReading) {
      emaSense2 = alpha * (currentReading) + (1 - alpha) * emaSense2;
    }
};
