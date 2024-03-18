#include <Wire.h>
#include <HardwareSerial.h>
#include "MagneticSensorI2C.h"
#include "MotorPWM.h"

// #if !( defined(STM32F1) )
// #error This code is designed to run on STM32F platform! Please check your Tools->Board setting.
// #endif

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
// HardwareSerial Serial3(PA10, PA9); // RX, TX
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

// Change the pin according to your STM32 board. There is no single definition for all boards
#define motor0A    PA6
#define motor0B    PA3
#define motor0C    PA2

MOTOR_PWM motor0 = MOTOR_PWM(motor0A, motor0B, motor0C);

//////////////////////////////////////////////////////

// SENSOR CLASSES
MagneticSensorI2C as5600_pitch = MagneticSensorI2C();
MagneticSensorI2C as5600_roll = MagneticSensorI2C();

//////////////////////////////////////////////////////

#define PRINT_INTERVAL    10000L

void setup()
{
  // INITIALIZE COMMUNICATION
  // Serial3.begin(9600);

  // I2C communication begin with proper pins
  i2c_1.begin(uint32_t(I2C_SDA_1), uint32_t(I2C_SCL_1));
  i2c_2.begin(uint32_t(I2C_SDA_2), uint32_t(I2C_SCL_2));

  motor0.setup();

  pinMode(LED_0, OUTPUT);
  pinMode(LED_1, OUTPUT);

  digitalWrite(LED_0, LED_OFF);
  digitalWrite(LED_1, LED_OFF);

  delay(1000);

  // Set up Encoder Sensors
  as5600_pitch.init(&i2c_2);
}

void loop()
{
  // IMPORTANT - call as frequently as possible
  // update the sensor values
  as5600_pitch.update();
  // float mechanicalAngle = as5600_pitch.getMechanicalAngle();
  
  float electricalAngle = as5600_pitch.getElectricalAngle();

  motor0.move(electricalAngle, false);

  // delay(1000);

  //display the angle and the angular velocity to the terminal
  // Serial3.print(mechanicalAngle);
  // Serial3.print("\t");
  // Serial3.print(electricalAngle);
  // Serial3.print("\t\n");
}