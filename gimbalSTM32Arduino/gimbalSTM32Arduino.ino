#include <Wire.h>
#include <HardwareSerial.h>
#include "MagneticSensorI2C.h"
#include "MotorPWM.h"
#include "adc.h"

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

//////////////////////////////////////////////////////

// SENSOR CLASSES
MagneticSensorI2C as5600_pitch = MagneticSensorI2C();
// MagneticSensorI2C as5600_roll = MagneticSensorI2C();

//////////////////////////////////////////////////////

#define PRINT_INTERVAL    10000L

// Function to read from a specific ADC input (GPIO pin)
uint16_t read_adc_for_pin(uint input) {
    adc_select_input(input); // Select the specific ADC input
    return adc_read(); // Perform a single conversion and return the result
}

void setup()
{
  // INITIALIZE COMMUNICATION
  delay(5000);

  Serial.begin(9600);

  adc_init();
  adc_gpio_init(MOTOR_ROLL_A_SENSE);
  adc_gpio_init(MOTOR_ROLL_B_SENSE);
  // Enable round-robin sampling for GPIO pins 26 and 27
  // adc_set_round_robin(0x03); // 0x03 is 3 in hexadecimal, representing bits 0 and 1 set.
  // Set ADC clock divider for desired sampling rate
  // adc_set_clkdiv(48000000.0f / 500000.0f - 1); // Example to achieve 500 kS/s

  // Setup the ADC FIFO for efficient sample handling
  // adc_fifo_setup(true, true, 3, true, false);

  Serial.println("ADC INITIALIZED");

  // // I2C communication begin with proper pins
  i2c_1.begin();
  // i2c_2.begin();

  rollMotor.setup();

  delay(5000);
  // pitchMotor.setup();

  delay(1000);

  // // Set up Encoder Sensors
  as5600_pitch.init(&i2c_1);
}

void loop()
{
  // Serial.println("Main Loop");
  uint16_t adc_value_28 = read_adc_for_pin(2); // For GPIO 26
  uint16_t adc_value_29 = read_adc_for_pin(3); // For GPIO 27
  // int adc_value_26 = analogRead(MOTOR_ROLL_A_SENSE); // For GPIO 26 equivalent
  // int adc_value_27 = analogRead(MOTOR_ROLL_B_SENSE); // For GPIO 27 equivalent
  // IMPORTANT - call as frequently as possible
  // update the sensor values
  as5600_pitch.update();
  
  float mechanicalAngle = as5600_pitch.getMechanicalAngle();
  float electricalAngle = as5600_pitch.getElectricalAngle() + 0.25f;
  
  // Print the read values to the Serial Monitor
  // Serial.print("RSENSE 0: ");
  Serial.print(adc_value_28);
  Serial.print(",");
  // Serial.print("RSENSE 1: ");
  Serial.println(adc_value_29);

  rollMotor.move(electricalAngle, false);
  // pitchMotor.move(electricalAngle, true);

  // delay(2000);

  //display the angle and the angular velocity to the terminal
  // Serial3.print(mechanicalAngle);
  // Serial3.print("\t");
  // Serial3.print(electricalAngle);
  // Serial3.print("\t\n");
}