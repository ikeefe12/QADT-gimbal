#include "MotorPWM.h"

// Constructor
MOTOR_PWM::MOTOR_PWM(uint32_t en, uint32_t aPWM, uint32_t bPWM, uint32_t cPWM) {
    enPin = en;
    
    pwmPins[0] = aPWM;
    pwmPins[1] = bPWM;
    pwmPins[2] = cPWM;

    // Duty cycles to align rotor with theta-electrical = 0
    dutyCycles[0] = 0.0f;
    dutyCycles[1] = 75.0f;
    dutyCycles[2] = 75.0f;
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
    PWM_Instance[index] = new RP2040_PWM(pwmPins[index], freq, dutyCycles[index]);

    if (PWM_Instance[index])
    {
      PWM_Instance[index]->setPWM();
    }
  }
}

void MOTOR_PWM::updateDutyCycles() {
  for (uint8_t index = 0; index < NUM_OF_PINS; index++)
  {
    PWM_Instance[index]->setPWM(pwmPins[index], freq, dutyCycles[index]);

  }
}

void MOTOR_PWM::move(float electricalAngle, bool ccw) {
    float dutyCycleA, dutyCycleB, dutyCycleC;
    float halfRange = 50.0f;

    float adjustAngle = ccw ? electricalAngle : -electricalAngle;

    // Adjust phase shifts based on rotation direction
    float phaseShiftB = ccw ? (4.0 * M_PI / 3.0) : (2.0 * M_PI / 3.0); // CW: Reverse B and C
    float phaseShiftC = ccw ? (2.0 * M_PI / 3.0) : (4.0 * M_PI / 3.0); // CCW: Original order

    // Phase A - no additional phase shift
    float sinA = sin(adjustAngle);
    dutyCycleA = sinA * halfRange + halfRange;
    
    // Phase B - Adjusted phase shift
    float sinB = sin(adjustAngle + phaseShiftB);
    dutyCycleB = sinB * halfRange + halfRange;
    
    // Phase C - Adjusted phase shift
    float sinC = sin(adjustAngle + phaseShiftC);
    dutyCycleC = sinC * halfRange + halfRange;

    dutyCycles[0] = dutyCycleA;
    dutyCycles[1] = dutyCycleB;
    dutyCycles[2] = dutyCycleC;
    
    // // Apply the calculated duty cycles to the PWM channels
    updateDutyCycles();
}
