#include "MotorPWM.h"

uint32_t freq = 100;
HardwareSerial Serial3 = HardwareSerial(PA10, PA9); // RX, TX

// Constructor
MOTOR_PWM::MOTOR_PWM(uint32_t phaseAPin, uint32_t phaseBpin, uint32_t phaseCpin) {
    Serial3.begin(9600);
    pins[0] = phaseAPin;
    pins[1] = phaseBpin;
    pins[2] = phaseCpin;
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
      if (index == 0) {
        timers[index]->setOverflow(freq, TICK_FORMAT);
      }
      
      timers[index]->setCaptureCompare(channels[index], dutyCycles[index], PERCENT_COMPARE_FORMAT);

      timers[index]->resumeChannel(channels[index]);
      
    }
    else
    {
      while (true)
            delay(100);
    }
  }
}

void MOTOR_PWM::updateDutyCycle(uint32_t motorIndex) {
    // Check if the motorIndex is valid
    if (motorIndex >= NUM_OF_PINS) {
        // Handle the error, perhaps by printing a message or returning early
        Serial3.println("Invalid motor index.");
        return;
    }

    // Check if the timer for this motorIndex has been initialized
    if (timers[motorIndex] != nullptr) {
        // Apply the new duty cycle using the setCaptureCompare method
        // Assuming the dutyCycle is given in a format compatible with the expected compare format
        // timers[motorIndex]->pauseChannel(channels[motorIndex]);
        timers[motorIndex]->setCaptureCompare(channels[motorIndex], dutyCycles[motorIndex], PERCENT_COMPARE_FORMAT);
        // timers[motorIndex]->resumeChannel(channels[motorIndex]);
    } else {
        // Handle the case where the timer hasn't been initialized
        Serial3.println("Timer not initialized for the specified motor index.");
    }
}

void MOTOR_PWM::move(float electricalAngle) {
    float dutyCycleA, dutyCycleB, dutyCycleC;
    float halfRange = float(maximumPWMDutyCycle / 2.0);

    // Phase A - no additional phase shift
    float sinA = sin(electricalAngle);
    dutyCycleA = sinA * halfRange + halfRange;
    Serial3.print(dutyCycleA);
    Serial3.print("\t");

    // Phase B - 120 degrees phase-shifted
    float sinB = sin(electricalAngle + (2.0 * M_PI / 3.0));
    dutyCycleB = sinB * halfRange + halfRange ;
    Serial3.print(dutyCycleB);
    Serial3.print("\t");

    // Phase C - 240 degrees phase-shifted
    float sinC = sin(electricalAngle + (4.0 * M_PI / 3.0));
    dutyCycleC = sinC * halfRange + halfRange;
    Serial3.print(dutyCycleC);
    Serial3.print("\t\n");
    
    // Convert float duty cycles to integer if required by updateDutyCycle method
    uint32_t intDutyCycleA = static_cast<uint32_t>(dutyCycleA);
    uint32_t intDutyCycleB = static_cast<uint32_t>(dutyCycleB);
    uint32_t intDutyCycleC = static_cast<uint32_t>(dutyCycleC);

    Serial3.print(intDutyCycleA);
    Serial3.print("\t");
    Serial3.print(intDutyCycleB);
    Serial3.print("\t");
    Serial3.print(intDutyCycleC);
    Serial3.print("\t\n");

    dutyCycles[0] = intDutyCycleA;
    dutyCycles[1] = intDutyCycleB;
    dutyCycles[2] = intDutyCycleC;
    
    // Apply the calculated duty cycles to the PWM channels
    updateDutyCycle(0); // Assuming motorIndex 0 corresponds to phase A
    updateDutyCycle(1); // Assuming motorIndex 1 corresponds to phase B
    updateDutyCycle(2); // Assuming motorIndex 2 corresponds to phase C
}


      // Serial3.print("Index = ");
      // Serial3.print(index);
      // Serial3.print(", Instance = 0x");
      // Serial3.print( (uint32_t) Instance, HEX);
      // Serial3.print(", channel = ");
      // Serial3.print(channels[index]);
      // Serial3.print(", TimerIndex = ");
      // Serial3.print(get_timer_index(Instance));
      // Serial3.print(", PinName = ");
      // Serial3.println( pinNameToUse );
