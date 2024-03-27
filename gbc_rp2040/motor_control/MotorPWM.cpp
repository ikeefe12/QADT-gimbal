#include "MotorPWM.h"

// Constructor
MOTOR_PWM::MOTOR_PWM(uint32_t en, uint32_t aPWM, uint32_t bPWM, uint32_t cPWM, uint32_t sense1, uint32_t sense2, int index1, int index2) {
    sense1Pin = sense1;
    sense2Pin = sense2;
    index_1 = index1;
    index_2 = index2;

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
  adc_gpio_init(sense1Pin);
  adc_gpio_init(sense2Pin);

  adc_select_input(index_1);
  adc_1_offset = adc_read();
  adc_select_input(index_2);
  adc_2_offset = adc_read();

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

// void MOTOR_PWM::updateCurrents() {
//   adc_select_input(index_1);
//   uint16_t sense1 = adc_read();
//   if (sense1 == 0) return;
//   sense1 = (sense1 > adc_1_offset) ? sense1 -  adc_1_offset : 0;
//   updateEMASense1(sense1);
//   float normalizedCurrent1 = emaSense1 / float(adc_1_max);

//   adc_select_input(index_2);
//   uint16_t sense2 = adc_read();
//   if (sense2 == 0) return;
//   sense2 = (sense2 > adc_2_offset) ? sense2 -  adc_2_offset : 0;
//   updateEMASense2(sense2);
//   float normalizedCurrent2 = emaSense2 / float(adc_2_max);

//   // CHECK THAT READS MAKE SENSE
//   float pwmA = sSVPWM.fCCRA;
//   float pwmB = sSVPWM.fCCRB;
//   float pwmC= sSVPWM.fCCRC;

//   if ((normalizedCurrent1 > 0.01) && (normalizedCurrent2 > 0.01)) {
//     // Readings from both sense pins
//     // Two negative phase currents, A||B and C
//     if (pwmA > pwmB) {
//       ia = normalizedCurrent1 + normalizedCurrent2;
//       ib = -normalizedCurrent1;
//     } else {
//       ib = normalizedCurrent1 + normalizedCurrent2;
//       ia = -normalizedCurrent1;
//     }
//     ic = -normalizedCurrent2;
//   } else if (normalizedCurrent2 > 0.01) {
//     // Readings from only sense2
//     pwmA -= pwmC;
//     pwmB -= pwmC;
//     float ratioA = pwmA / (pwmA + pwmB);
//     float ratioB = pwmB / (pwmA + pwmB);
//     ia = ratioA * normalizedCurrent2;
//     ib = ratioB * normalizedCurrent2;
//     ic = -normalizedCurrent2;
//   } else if (normalizedCurrent1 > 0.01) {
//     // Readings from only sense1
//     if (pwmA > pwmB) {
//       float normalizedPWMA = pwmA / (pwmC - pwmB);
//       if (normalizedPWMA > 0.5f) {
//         // Phase A and C have positive currents
//         // Phase B is only crontibuter to negative current in sense1
//         pwmA -= pwmB;
//         pwmC -= pwmB;
//         float ratioA = pwmA / (pwmA + pwmC);
//         float ratioC = pwmC / (pwmA + pwmC);
//         ia = ratioA * normalizedCurrent1;
//         ic = ratioC * normalizedCurrent1;
//         ib = -normalizedCurrent1;
//       } else {
//         // Phase C has positive current
//         // Phase A and B are contributing to negative current in sense1
//         pwmA = pwmC - pwmA;
//         pwmB = pwmC - pwmB;
//         float ratioA = pwmA / (pwmA + pwmB);
//         float ratioB = pwmB / (pwmA + pwmB);
//         ia = ratioA * -normalizedCurrent1;
//         ib = ratioB * -normalizedCurrent1;
//         ic = normalizedCurrent1;
//       }
//     } else {
//       float normalizedPWMB = pwmB / (pwmC - pwmA);
//       if (normalizedPWMB > 0.5f) {
//         // Phase B and C have positive currents
//         // Phase A is only crontibuter to negative current in sense1
//         pwmB -= pwmA;
//         pwmC -= pwmA;
//         float ratioB = pwmB / (pwmB + pwmC);
//         float ratioC = pwmC / (pwmB + pwmC);
//         ib = ratioB * normalizedCurrent1;
//         ic = ratioC * normalizedCurrent1;
//         ia = -normalizedCurrent1;
//       } else {
//         // Phase C has positive current
//         // Phase A and B are contributing to negative current in sense1
//         pwmA = pwmC - pwmA;
//         pwmB = pwmC - pwmB;
//         float ratioA = pwmA / (pwmA + pwmB);
//         float ratioB = pwmB / (pwmA + pwmB);
//         ia = ratioA * -normalizedCurrent1;
//         ib = ratioB * -normalizedCurrent1;
//         ic = normalizedCurrent1;
//       }
//     }
//   }
// }

void MOTOR_PWM::updateReadings() {
  adc_select_input(index_1);
  uint16_t sense1 = adc_read();
  if (sense1 == 0) return;
  sense1 = (sense1 > adc_1_offset) ? sense1 -  adc_1_offset : 0;
  updateEMASense1(sense1);

  adc_select_input(index_2);
  uint16_t sense2 = adc_read();
  if (sense2 == 0) return;
  sense2 = (sense2 > adc_2_offset) ? sense2 -  adc_2_offset : 0;
  updateEMASense2(sense2);
}

void MOTOR_PWM::updateCurrents() {
  float normalizedCurrent1 = emaSense1 / float(adc_1_max);
  float normalizedCurrent2 = emaSense2 / float(adc_2_max);

  float pwmA = sSVPWM.fCCRA;
  float pwmB = sSVPWM.fCCRB;
  float pwmC= sSVPWM.fCCRC;

  if (pwmA < pwmB && pwmB < pwmC) {
    // Only use sense 1
    // A < B < C
    float bRange = (pwmB - pwmA) / (pwmC - pwmA);
    if (bRange > 0.5f) { 
      // B and C are positive currents
      // A is negative current indicated by sense 1

    } else {
      // C is positive current
      // A and B are negative currents contributing to sense 1

    }
  } else if (pwmB < pwmA && pwmA < pwmC) {
    // Only use sense 1
    // B < A < C
    float aRange = (pwmA - pwmB) / (pwmC - pwmB);
    if (aRange > 0.5f) { 
      // A and C are positive currents
      // B is negative current indicated by sense 1

    } else {
      // C is positive current
      // A and B are negative currents contributing to sense 1

    }
  } else if (pwmB < pwmC && pwmC < pwmA) {
    // B < C < A
    float cRange = (pwmC - pwmB) / (pwmA - pwmB);
    if (cRange > 0.5f) { 
      // Only use sense 1
      // A and C are positive currents
      // B is negative current indicated by sense 1

    } else {
      // C is negative current indicated by sense2
      // B is negative current indicated by sense1
      // A is positive current
      ia = normalizedCurrent1 + normalizedCurrent2;
      ib = -normalizedCurrent1;
      ic = -normalizedCurrent2;
    }
  } else if (pwmC < pwmB && pwmB < pwmA) {
    float bRange = (pwmB - pwmC) / (pwmA - pwmC);
    if (bRange > 0.5f) { 
      // Only use sense 2
      // A and B are positive currents
      // C is negative current indicated by sense 2

    } else {
      // C is negative current indicated by sense2
      // B is negative current indicated by sense1
      // A is positive current
      ia = normalizedCurrent1 + normalizedCurrent2;
      ib = -normalizedCurrent1;
      ic = -normalizedCurrent2;
    }
      // C < B < A
  } else if (pwmC < pwmA && pwmA < pwmB) {
    // C < A < B
    float aRange = (pwmA - pwmC) / (pwmB - pwmC);

    if (aRange > 0.5f) { 
      // Only use sense 2
      // A and B are positive currents
      // C is negative current indicated by sense 2

    } else {
      // C is negative current indicated by sense2
      // A is negative current indicated by sense1
      // B is positive current
      ib = normalizedCurrent1 + normalizedCurrent2;
      ia = -normalizedCurrent1;
      ic = -normalizedCurrent2;
    }
  } else if (pwmA < pwmC && pwmC < pwmB) {
      // A < C < B
      float cRange = (pwmC - pwmA) / (pwmB - pwmA);

      if (cRange > 0.5f) { 
        // Only use sense 1
        // B and C are positive currents
        // A is negative current indicated by sense 1

      } else {
        // C is negative current indicated by sense2
        // A is negative current indicated by sense1
        // B is positive current
        ib = normalizedCurrent1 + normalizedCurrent2;
        ia = -normalizedCurrent1;
        ic = -normalizedCurrent2;
      }
  }
}

void MOTOR_PWM::updateCurrents() {
  float normalizedCurrent1 = emaSense1 / float(adc_1_max);
  float normalizedCurrent2 = emaSense2 / float(adc_2_max);

  // CHECK THAT READS MAKE SENSE
  float pwmA = sSVPWM.fCCRA;
  float pwmB = sSVPWM.fCCRB;
  float pwmC= sSVPWM.fCCRC;

  if (normalizedCurrent2 > normalizedCurrent1) {
    // ensure the readings make logical sense
    if ((pwmC > pwmA) || (pwmC > pwmB)) return;

    float aRange = (pwmA - pwmC) / (pwmB - pwmC);
    float bRange = (pwmB - pwmC) / (pwmA - pwmC);

    if ((aRange > 0.5f) && (bRange > 0.5f)) {
      // Readings from only sense2 are valid
      // both A and B are positive currents
      pwmA -= pwmC;
      pwmB -= pwmC;
      float ratioA = pwmA / (pwmA + pwmB);
      float ratioB = pwmB / (pwmA + pwmB);
      ia = ratioA * normalizedCurrent2;
      ib = ratioB * normalizedCurrent2;
      ic = -normalizedCurrent2;
    } else if (pwmA > pwmB) {
      // Readings from both sense pins are valid
      // A is positive
      // B/C negative
      ia = normalizedCurrent1 + normalizedCurrent2;
      ib = -normalizedCurrent1;
      ic = -normalizedCurrent2;
    } else {
      // Readings from both sense pins are valid
      // B is positive
      // A/C negative
      ib = normalizedCurrent1 + normalizedCurrent2;
      ia = -normalizedCurrent1;
      ic = -normalizedCurrent2;
    }
  } else { // normalizedCurrent1 > normalizedCurrent2
    // ensure the readings make logical sense
    if ((pwmC < pwmA) && (pwmC < pwmB)) return;

    float cRange = (pwmA > pwmB) ? (pwmC - pwmB) / (pwmA - pwmB) : (pwmC - pwmA) / (pwmB - pwmA);

    if (cRange > 0.5f) {
      // Readings from only sense1 are valid
      if (pwmA > pwmB) {
        float distA = (pwmA - pwmB) / (pwmC - pwmB);
        if (distA > 0.5f) { // positive A current
          // Phase A and C have positive currents
          // Phase B is only crontibuter to negative current in sense1
          pwmA -= pwmB;
          pwmC -= pwmB;
          float ratioA = pwmA / (pwmA + pwmC);
          float ratioC = pwmC / (pwmA + pwmC);
          ia = ratioA * normalizedCurrent1;
          ic = ratioC * normalizedCurrent1;
          ib = -normalizedCurrent1;
        } else { // negative A current
          // Phase C has positive current
          // Phase A and B are contributing to negative current in sense1
          pwmA = pwmC - pwmA;
          pwmB = pwmC - pwmB;
          float ratioA = pwmA / (pwmA + pwmB);
          float ratioB = pwmB / (pwmA + pwmB);
          ia = ratioA * -normalizedCurrent1;
          ib = ratioB * -normalizedCurrent1;
          ic = normalizedCurrent1;
        }
      } else { // pwmB > pwmA
        float distB = (pwmB - pwmA) / (pwmC - pwmA);
        if (distB > 0.5f) { // positive B current
          // Phase B and C have positive currents
          // Phase A is only crontibuter to negative current in sense1
          pwmB -= pwmA;
          pwmC -= pwmA;
          float ratioB = pwmB / (pwmB + pwmC);
          float ratioC = pwmC / (pwmB + pwmC);
          ib = ratioB * normalizedCurrent1;
          ic = ratioC * normalizedCurrent1;
          ia = -normalizedCurrent1;
        } else { // negative B current
          pwmA = pwmC - pwmA;
          pwmB = pwmC - pwmB;
          float ratioA = pwmA / (pwmA + pwmB);
          float ratioB = pwmB / (pwmA + pwmB);
          ia = ratioA * -normalizedCurrent1;
          ib = ratioB * -normalizedCurrent1;
          ic = normalizedCurrent1;
        }
      }
    } else if (pwmA > pwmB) {
      ia = normalizedCurrent1 + normalizedCurrent2;
      ib = -normalizedCurrent1;
      ic = -normalizedCurrent2;
    } else {
      ib = normalizedCurrent1 + normalizedCurrent2;
      ia = -normalizedCurrent1;
      ic = -normalizedCurrent2;
    }
  }
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
