#include <Wire.h>
#include <HardwareSerial.h>
#include "MagneticSensorI2C.h"
#include "MotorPWM.h"

extern "C" {
  #include "hardware/i2c.h"
}

// I2C PINS
// I2C 1 is located next to LEDs on STorM32
// I2C 1 Used for IMU and Magnetic encoder #1
#define I2C_SCL_1 17
#define I2C_SDA_1 16
// I2C 2 Used for Magnetic encoder #2
// #define I2C_SCL_2 3
// #define I2C_SDA_2 4

/////////////////////////////////////////////////
// SERIAL / I2C
// HardwareSerial Serial3(PA10, PA9); // RX, TX
TwoWire i2c_1(i2c0, I2C_SDA_1, I2C_SCL_1);
// TwoWire i2c_2 = TwoWire(uint32_t(I2C_SDA_2), uint32_t(I2C_SCL_2));
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
#define MPU_addr 0x68

/////////////////////////////////////////////////

// Change the pin according to your STM32 board. There is no single definition for all boards
#define MOTOR_PITCH_EN    9
#define MOTOR_PITCH_A_PWM    10
#define MOTOR_PITCH_B_PWM    11
#define MOTOR_PITCH_C_PWM    12

MOTOR_PWM pitchMotor = MOTOR_PWM(
  MOTOR_PITCH_EN,
  MOTOR_PITCH_A_PWM,
  MOTOR_PITCH_B_PWM,
  MOTOR_PITCH_C_PWM
);

//////////////////////////////////////////////////////

// SENSOR CLASSES
MagneticSensorI2C as5600_pitch = MagneticSensorI2C();
// MagneticSensorI2C as5600_roll = MagneticSensorI2C();

//////////////////////////////////////////////////////

#define PRINT_INTERVAL    10000L

void setup()
{
  // INITIALIZE COMMUNICATION
  Serial.begin(9600);

  // // I2C communication begin with proper pins
  i2c_1.begin();
  // i2c_2.begin();

  pitchMotor.setup();

  delay(1000);

  // // Set up Encoder Sensors
  as5600_pitch.init(&i2c_1);
}

void loop()
{
  Serial.println("Main Loop");
  // IMPORTANT - call as frequently as possible
  // update the sensor values
  as5600_pitch.update();
  
  float mechanicalAngle = as5600_pitch.getMechanicalAngle();
  float electricalAngle = as5600_pitch.getElectricalAngle();
  
  Serial.println(mechanicalAngle);

  pitchMotor.move(electricalAngle, false);

  //display the angle and the angular velocity to the terminal
  // Serial3.print(mechanicalAngle);
  // Serial3.print("\t");
  // Serial3.print(electricalAngle);
  // Serial3.print("\t\n");
}