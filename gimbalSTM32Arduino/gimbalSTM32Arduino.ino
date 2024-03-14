#if !( defined(STM32F1) )
#error This code is designed to run on STM32F platform! Please check your Tools->Board setting.
#endif

// These define's must be placed at the beginning before #include "ESP32_PWM.h"
// _PWM_LOGLEVEL_ from 0 to 4
// Don't define _PWM_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define _PWM_LOGLEVEL_      0

#define USING_MICROS_RESOLUTION       true    //false 

#include "STM32_PWM.h"
#include <Wire.h>
#include <HardwareSerial.h>
#include "MagneticSensorI2C.h"

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
TwoWire i2c_1;
TwoWire i2c_2;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
#define MPU_addr 0x68

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

// SENSOR CLASSES
MagneticSensorI2C as5600_pitch = MagneticSensorI2C();
MagneticSensorI2C as5600_roll = MagneticSensorI2C();

/////////////////////////////////////////////////////

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

  // I2C communication begin with proper pins
  i2c_1.begin(uint32_t(I2C_SDA_1), uint32_t(I2C_SCL_1));
  i2c_2.begin(uint32_t(I2C_SDA_2), uint32_t(I2C_SCL_2));

  // Set up Encoder Sensors
  as5600_pitch.init(&i2c_2);

  i2c_2.beginTransmission(MPU_addr);
  i2c_2.write(0x6B);  // PWR_MGMT_1 register
  i2c_2.write(0);     // set to zero (wakes up the MPU-6050)
  i2c_2.endTransmission(true);

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
}

void loop()
{
  // IMPORTANT - call as frequently as possible
  // update the sensor values
  float rotorAngle = as5600_pitch.getSensorAngle(); // - as5600Offset;
  // display the angle and the angular velocity to the terminal
  Serial3.print(rotorAngle);
  Serial3.print("\t\n");

  i2c_2.beginTransmission(MPU_addr);
  i2c_2.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  i2c_2.endTransmission(false);
  i2c_2.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=i2c_2.read()<<8|i2c_2.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=i2c_2.read()<<8|i2c_2.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=i2c_2.read()<<8|i2c_2.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=i2c_2.read()<<8|i2c_2.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=i2c_2.read()<<8|i2c_2.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=i2c_2.read()<<8|i2c_2.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=i2c_2.read()<<8|i2c_2.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial3.print("AcX = "); Serial3.print(AcX);
  Serial3.print(" | AcY = "); Serial3.print(AcY);
  Serial3.print(" | AcZ = "); Serial3.print(AcZ);
  Serial3.print(" | Tmp = "); Serial3.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial3.print(" | GyX = "); Serial3.print(GyX);
  Serial3.print(" | GyY = "); Serial3.print(GyY);
  Serial3.print(" | GyZ = "); Serial3.println(GyZ);
  delay(10);
}