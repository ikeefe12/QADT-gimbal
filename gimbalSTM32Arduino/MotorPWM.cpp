#include "MotorPWM.h"

HardwareSerial Serial3 = HardwareSerial(PA10, PA9); // RX, TX

// Constructor
MOTOR_PWM::MOTOR_PWM(uint32_t phaseAPin, uint32_t phaseBpin, uint32_t phaseCpin) {
    Serial3.begin(9600);
    
    pins[0] = phaseAPin;
    pins[1] = phaseBpin;
    pins[2] = phaseCpin;

    // Duty cycles to align rotor with theta-electrical = 0
    dutyCycles[0] = 0;
    dutyCycles[1] = 49151;
    dutyCycles[2] = 49151;
}

void MOTOR_PWM::setup() {
  for (uint8_t index = 0; index < NUM_OF_PINS; index++)
  {
    pinMode(pins[index], OUTPUT);
    digitalWrite(pins[index], LOW);
  }

  for (uint8_t index = 0; index < NUM_OF_PINS; index++)
  {
    // Using pin = PA0, PA1, etc.
    PinName pinNameToUse = digitalPinToPinName(pins[index]);

    // Automatically retrieve TIM instance and channel associated to pin
    // This is used to be compatible with all STM32 series automatically.
    TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pinNameToUse, PinMap_PWM);

    if (Instance != nullptr)
    {
      // pin => 0, 1, etc
      channels[index] = STM_PIN_CHANNEL(pinmap_function( pinNameToUse, PinMap_PWM));
      timers[index] = new HardwareTimer(Instance);
      timers[index]->setMode(channels[index], TIMER_OUTPUT_COMPARE_PWM1, pins[index]);
      // if (index == 0) {
      timers[index]->setOverflow(freq, HERTZ_FORMAT);
      // }
      
      timers[index]->setCaptureCompare(channels[index], dutyCycles[index], RESOLUTION_16B_COMPARE_FORMAT);

      timers[index]->resumeChannel(channels[index]);
      
    }
    else
    {
      while (true)
            delay(100);
    }
  }
}

void MOTOR_PWM::updateDutyCycles() {
  for (uint8_t index = 0; index < NUM_OF_PINS; index++)
  {
    timers[index]->setCaptureCompare(channels[index], dutyCycles[index], RESOLUTION_16B_COMPARE_FORMAT);
  }
}

void MOTOR_PWM::move(float electricalAngle, bool ccw) {
    float dutyCycleA, dutyCycleB, dutyCycleC;
    float halfRange = float(maximumPWMDutyCycle / 2.0);

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
    
    // Convert float duty cycles to integer if required by updateDutyCycle method
    uint32_t intDutyCycleA = static_cast<uint32_t>(dutyCycleA);
    uint32_t intDutyCycleB = static_cast<uint32_t>(dutyCycleB);
    uint32_t intDutyCycleC = static_cast<uint32_t>(dutyCycleC);

    dutyCycles[0] = intDutyCycleA;
    dutyCycles[1] = intDutyCycleB;
    dutyCycles[2] = intDutyCycleC;
    
    // Apply the calculated duty cycles to the PWM channels
    updateDutyCycles();
}
