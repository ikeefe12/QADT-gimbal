#include <Wire.h>
#include <HardwareSerial.h>
#include "sensors/MagneticSensorI2C.h"
#include "sensors/MagneticSensorI2C.cpp"
#include "sensors/CurrentSense.h"
#include "sensors/CurrentSense.cpp"
#include "motor_control/MotorPWM.h"
#include "motor_control/MotorPWM.cpp"

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
#define MOTOR_PITCH_A_SENSE    26
#define MOTOR_PITCH_B_PWM    11
#define MOTOR_PITCH_B_SENSE   27
#define MOTOR_PITCH_C_PWM    12

#define MOTOR_ROLL_EN    5
#define MOTOR_ROLL_A_PWM    6
#define MOTOR_ROLL_A_SENSE    28
#define MOTOR_ROLL_B_PWM    7
#define MOTOR_ROLL_B_SENSE   29
#define MOTOR_ROLL_C_PWM    8
// phase C current not used

MOTOR_PWM pitchMotor = MOTOR_PWM(
  MOTOR_PITCH_EN,
  MOTOR_PITCH_A_PWM,
  MOTOR_PITCH_B_PWM,
  MOTOR_PITCH_C_PWM
);

MOTOR_PWM rollMotor = MOTOR_PWM(
  MOTOR_ROLL_EN,
  MOTOR_ROLL_A_PWM,
  MOTOR_ROLL_B_PWM,
  MOTOR_ROLL_C_PWM
);

CurrentSense rollMotorCurrent = CurrentSense(
  MOTOR_ROLL_A_SENSE,
  MOTOR_ROLL_B_SENSE,
  2,
  3
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

  delay(1000);

  rollMotorCurrent.setup();
  
  // // I2C communication begin with proper pins
  i2c_1.begin();
  // i2c_2.begin();

  rollMotor.setup();
  // pitchMotor.setup();

  delay(1000);

  // // Set up Encoder Sensors
  as5600_pitch.init(&i2c_1);
}

void loop()
{
  // Serial.println("Main Loop");
  
  // IMPORTANT - call as frequently as possible
  // update the sensor values
  as5600_pitch.update();
  rollMotorCurrent.update();
  
  float mechanicalAngle = as5600_pitch.getMechanicalAngle();

  float electricalAngle = as5600_pitch.getElectricalAngle() + 0.75;

  Serial.print(rollMotorCurrent.ia);
  Serial.print(",");
  Serial.println(rollMotorCurrent.ib);

  rollMotor.move(electricalAngle, false);
  // pitchMotor.move(electricalAngle, true);

  delay(5);
}