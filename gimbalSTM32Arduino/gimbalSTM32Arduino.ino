#if !( defined(STM32F1) )
#error This code is designed to run on STM32F platform! Please check your Tools->Board setting.
#endif

// These define's must be placed at the beginning before #include "ESP32_PWM.h"
// _PWM_LOGLEVEL_ from 0 to 4
// Don't define _PWM_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define _PWM_LOGLEVEL_      0

#define USING_MICROS_RESOLUTION       true    //false 

#include "STM32_PWM.h"
#include "MagneticSensorI2C.h"
#include <HardwareSerial.h>

#define LED_ON        LOW
#define LED_OFF       HIGH

#ifndef LED_0
  #define LED_0       PB12
#endif

#ifndef LED_1
  #define LED_1          PB13
#endif

/////////////////////////////////////////////////
// SERIAL / I2C
HardwareSerial Serial3(PA10, PA9); // RX, TX
MagneticSensorI2C as5600 = MagneticSensorI2C();
// float as5600Offset = 0;

/////////////////////////////////////////////////

// I2C PINS
// I2C 1 is located next to LEDs on STorM32
// I2C 1 Used for IMU and Magnetic encoder #1
#define I2C_SCL_1 PB10
#define I2C_SDA_1 PB11
// I2C 2 Used for Magnetic encoder #2
#define I2C_SCL_2 PB6
#define I2C_SDA_2 PB7

// Change the pin according to your STM32 board. There is no single definition for all boards.
#define motor0Poles    14
#define motor0A    PB1
#define motor0B    PB0
#define motor0C    PA7

//////////////////////////////////////////////////////

// motor0 Pins  ============>> TimerIndex = 1, 0, 3
uint32_t pins[]       = { motor0A, motor0B, motor0C };

//////////////////////////////////////////////////////

#define NUM_OF_PINS       ( sizeof(pins) / sizeof(uint32_t) )

#define NUMBER_ISR_PWMS         NUM_OF_PINS

uint32_t dutyCycle[NUM_OF_PINS]  = { 127, 237, 17 };

uint32_t freq[] = { 1, 1, 1 };

TIM_TypeDef *TimerInstance[] = { TIM3, TIM3, TIM3 };

uint32_t callbackTime[] = { 0, 0, 0 };

//////////////////////////////////////////////////////

typedef void (*timerPWMCallback)  (uint32_t* data);

void PeriodCallback0(uint32_t* cbTime)
{
  *cbTime = *cbTime + 1;
}


void PeriodCallback1(uint32_t* cbTime)
{
  *cbTime = *cbTime + 1;
}

void PeriodCallback2(uint32_t* cbTime)
{
  *cbTime = *cbTime + 1;
}

//////////////////////////////////////////////////////

timerPWMCallback PeriodCallback[] =
{
  PeriodCallback0,  PeriodCallback1,  PeriodCallback2
};

//////////////////////////////////////////////////////

//////////////////////////////////////////////////////

#define PRINT_INTERVAL    10000L

void setup()
{
  // INITIALIZE COMMUNICATION
  Serial3.begin(9600);
  as5600.init(I2C_SDA_2, I2C_SCL_2);

  pinMode(LED_0, OUTPUT);
  pinMode(LED_1, OUTPUT);

  digitalWrite(LED_0, LED_OFF);
  digitalWrite(LED_1, LED_OFF);

  for (uint8_t index = 0; index < NUM_OF_PINS; index++)
  {
    pinMode(pins[index], OUTPUT);
    digitalWrite(pins[index], LOW);
  }

  for (uint8_t index = 0; index < NUM_OF_PINS; index++)
  {
    uint16_t Timer_Used[NUM_OF_PINS];

    // Set unused TimerIndex = 0xFFFF
    memset(Timer_Used, 0xFFFF, sizeof(Timer_Used));

    // Using pin = PA0, PA1, etc.
    PinName pinNameToUse = digitalPinToPinName(pins[index]);

    // Automatically retrieve TIM instance and channel associated to pin
    // This is used to be compatible with all STM32 series automatically.
    TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pinNameToUse, PinMap_PWM);

    if (Instance != nullptr)
    {
      uint8_t timerIndex = get_timer_index(Instance);

      // pin => 0, 1, etc
      uint32_t channel = STM_PIN_CHANNEL(pinmap_function( pinNameToUse, PinMap_PWM));

      for ( uint8_t i = 0; i < index; i++)
      {
        if (timerIndex == Timer_Used[i])
        {
          while (true)
            delay(100);
        }
      }

      // Update Timer_Used with current timerIndex
      Timer_Used[index] = timerIndex;

      HardwareTimer *MyTim = new HardwareTimer(Instance);

      MyTim->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, pins[index]);
      MyTim->setOverflow(freq[index], TICK_FORMAT);
      MyTim->setCaptureCompare(channel, dutyCycle[index], PERCENT_COMPARE_FORMAT);

      if (PeriodCallback[index])
      {
        MyTim->attachInterrupt( std::bind(PeriodCallback[index], &callbackTime[index]) );
      }

      MyTim->resume();
    }
    else
    {
      while (true)
            delay(100);
    }
  }

  delay(100);
  
  // as5600.update();

  // as5600Offset = as5600.getAngle();
}

void loop()
{
  // IMPORTANT - call as frequently as possible
  // update the sensor values
  as5600.update();
  float rotorAngle = as5600.getAngle(); // - as5600Offset;
  // display the angle and the angular velocity to the terminal
  Serial3.print(rotorAngle);
  Serial3.print("\t\n");
  delay(10);
}