#include "MagneticSensorI2C.h"

// function buffering delay() 
// arduino uno function doesn't work well with interrupts
void _delay(unsigned long ms){
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328PB__)  || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__)
  // if arduino uno and other atmega328p chips
  // use while instad of delay, 
  // due to wrong measurement based on changed timer0
  unsigned long t = _micros() + ms*1000;
  while( _micros() < t ){}; 
#else
  // regular micros
  delay(ms);
#endif
}


// function buffering _micros() 
// arduino function doesn't work well with interrupts
unsigned long _micros(){
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega328PB__)  || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega32U4__)
// if arduino uno and other atmega328p chips
    //return the value based on the prescaler
    if((TCCR0B & 0b00000111) == 0x01) return (micros()/32);
    else return (micros());
#else
  // regular micros
  return micros();
#endif
}

MagneticSensorI2C::MagneticSensorI2C(){
  chip_address = 0x36; 

  // angle read register of the magnetic sensor
  angle_register_msb = 0x0C;
  // register maximum value (counts per revolution)
  cpr = _powtwo(12);

  int bits_used_msb = 11 - 7;
  lsb_used = 12 - bits_used_msb;
  // extraction masks
  lsb_mask = (uint8_t)( (2 << lsb_used) - 1 );
  msb_mask = (uint8_t)( (2 << bits_used_msb) - 1 );

  wire = &Wire;
}

void MagneticSensorI2C::update() {
    float val = getSensorAngle();
    if (val<0) // sensor angles are strictly non-negative. Negative values are used to signal errors.
         return;
    angle_prev = val;
}

float MagneticSensorI2C::getAngle(){
    return angle_prev;
}

void MagneticSensorI2C::init(uint32_t sdaPin, uint32_t sclPin, TwoWire* _wire){
  wire = _wire;

  // I2C communication begin
  wire->begin(sdaPin, sclPin);

  // initialize all the internal variables of Sensor to ensure a "smooth" startup (without a 'jump' from zero)
  update(); // call once
  getSensorAngle(); // call once
  delayMicroseconds(1);
  update();
  angle_prev = getSensorAngle(); // call again
}

//  Shaft angle calculation
//  angle is in radians [rad]
float MagneticSensorI2C::getSensorAngle(){
  // (number of full rotations)*2PI + current sensor angle 
  return  ( getRawCount() / (float)cpr) * _2PI ;
}



// function reading the raw counter of the magnetic sensor
int MagneticSensorI2C::getRawCount(){
	return (int)MagneticSensorI2C::read(angle_register_msb);
}

// I2C functions
/*
* Read a register from the sensor
* Takes the address of the register as a uint8_t
* Returns the value of the register
*/
int MagneticSensorI2C::read(uint8_t angle_reg_msb) {
  // read the angle register first MSB then LSB
	byte readArray[2];
	uint16_t readValue = 0;
  // notify the device that is aboout to be read
	wire->beginTransmission(chip_address);
	wire->write(angle_reg_msb);
  currWireError = wire->endTransmission(false);

  // read the data msb and lsb
	wire->requestFrom(chip_address, (uint8_t)2);
	for (byte i=0; i < 2; i++) {
		readArray[i] = wire->read();
	}

  // depending on the sensor architecture there are different combinations of
  // LSB and MSB register used bits
  // AS5600 uses 0..7 LSB and 8..11 MSB
  // AS5048 uses 0..5 LSB and 6..13 MSB
  readValue = ( readArray[1] &  lsb_mask );
	readValue += ( ( readArray[0] & msb_mask ) << lsb_used );
	return readValue;
}
