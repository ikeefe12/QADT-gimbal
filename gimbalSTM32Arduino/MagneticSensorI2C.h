#ifndef MAGNETICSENSORI2C_LIB_H
#define MAGNETICSENSORI2C_LIB_H

#include "Arduino.h"
#include <Wire.h>

#define _powtwo(x) (1 << (x))
#define _2PI 6.28318530718f

/** 
 * Function implementing delay() function in milliseconds 
 * - blocking function
 * - hardware specific

 * @param ms number of milliseconds to wait
 */
void _delay(unsigned long ms);

/** 
 * Function implementing timestamp getting function in microseconds
 * hardware specific
 */
unsigned long _micros();


class MagneticSensorI2C {
 public:
    // hard coded as both encoders use the same address
    MagneticSensorI2C();
        
    /** sensor initialise pins */
    void init(TwoWire* _wire);

    //void update();

    // float getAngle();

    // implementation of abstract functions of the Sensor class
    /** get current angle (rad) */
    float getSensorAngle();

    /** current error code from Wire endTransmission() call **/
    uint8_t currWireError = 0;
    float angle_prev=0.0f; // result of last call to getSensorAngle()

  private:
    float cpr; //!< Maximum range of the magnetic sensor
    uint16_t lsb_used; //!< Number of bits used in LSB register
    uint8_t lsb_mask;
    uint8_t msb_mask;
    
    // I2C variables
    uint8_t angle_register_msb; //!< I2C angle register to read
    uint8_t chip_address; //!< I2C chip select pins

    // I2C functions
    /** Read one I2C register value */
    int read(uint8_t angle_register_msb);

    /**
     * Function getting current angle register value
     * it uses angle_register variable
     */
    int getRawCount();
    
    /* the two wire instance for this sensor */
    TwoWire* wire;
};


#endif
