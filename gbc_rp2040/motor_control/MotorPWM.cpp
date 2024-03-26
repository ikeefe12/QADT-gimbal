#include "MotorPWM.h"

// Constructor
MOTOR_PWM::MOTOR_PWM(uint32_t en, uint32_t aPWM, uint32_t bPWM, uint32_t cPWM) {
    enPin = en;
    
    pwmPins[0] = aPWM;
    pwmPins[1] = bPWM;
    pwmPins[2] = cPWM;

    // Duty cycles to align rotor with theta-electrical = 0
    sSVPWM.enInType = UsAng;
    sSVPWM.fUdc = 16.0f;    // set the DC-Link voltage in Volts
    sSVPWM.fUdcCCRval = maximumPWMDutyCycle; // set the Max value of counter compare register which equal to DC-Link voltage
    sSVPWM.fUs = volatageMagintude;	// set dynamically in future - volatageMagintude
}

void MOTOR_PWM::setup() {
  // Initialize enable pin to high
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, HIGH);

  for (uint8_t index = 0; index < NUM_OF_PINS; index++)
  {
    // Initialize pwm pin
    pinMode(pwmPins[index], OUTPUT);
    digitalWrite(pwmPins[index], LOW);
  }

  for (uint8_t index = 0; index < NUM_OF_PINS; index++)
  {
    PWM_Instance[index] = new RP2040_PWM(pwmPins[index], freq, initialDutyCycles[index]);

    if (PWM_Instance[index])
    {
      PWM_Instance[index]->setPWM();
    }
  }
}

void MOTOR_PWM::updateDutyCycles() {
    PWM_Instance[0]->setPWM(pwmPins[0], freq, sSVPWM.fCCRA);
    PWM_Instance[1]->setPWM(pwmPins[1], freq, sSVPWM.fCCRB);
    PWM_Instance[2]->setPWM(pwmPins[2], freq, sSVPWM.fCCRC);
}

// Demo purposes to make sure motor runs
void MOTOR_PWM::move(float electricalAngle, bool cw) {
    // float adjustAngle = cw ? electricalAngle : -electricalAngle;

    // Ensure the magnitude of angle is less than 2pi
    sSVPWM.fAngRad = fmod(electricalAngle, 2.0 * M_PI);;	// set a new value of voltage Beta
    // Ensure the result is always positive
    if (sSVPWM.fAngRad < 0) {
        sSVPWM.fAngRad += 2.0 * M_PI;
    }

    sSVPWM.m_calc(&sSVPWM);
    
    // // Apply the calculated duty cycles to the PWM channels
    updateDutyCycles();
}

// void MOTOR_PWM::move(float electricalAngle, bool ccw) {
    // float dutyCycleA, dutyCycleB, dutyCycleC;
    // float halfRange = 50.0f;

    // float adjustAngle = ccw ? electricalAngle : -electricalAngle;

    // // Adjust phase shifts based on rotation direction
    // float phaseShiftB = ccw ? (4.0 * M_PI / 3.0) : (2.0 * M_PI / 3.0); // CW: Reverse B and C
    // float phaseShiftC = ccw ? (2.0 * M_PI / 3.0) : (4.0 * M_PI / 3.0); // CCW: Original order

    // // Phase A - no additional phase shift
    // float sinA = sin(adjustAngle);
    // dutyCycleA = sinA * halfRange + halfRange;
    
    // // Phase B - Adjusted phase shift
    // float sinB = sin(adjustAngle + phaseShiftB);
    // dutyCycleB = sinB * halfRange + halfRange;
    
    // // Phase C - Adjusted phase shift
    // float sinC = sin(adjustAngle + phaseShiftC);
    // dutyCycleC = sinC * halfRange + halfRange;

    // dutyCycles[0] = dutyCycleA;
    // dutyCycles[1] = dutyCycleB;
    // dutyCycles[2] = dutyCycleC;
    
    // // // Apply the calculated duty cycles to the PWM channels
    // updateDutyCycles();
// }
