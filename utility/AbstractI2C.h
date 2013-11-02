#ifndef __I2C_H__
#define __I2C_H__

#include <stdint.h>

class AbstractI2C {
public:
  AbstractI2C() {}
  virtual ~AbstractI2C() {}

  virtual uint8_t write(uint8_t address, uint8_t reg, uint8_t value) = 0;
  virtual uint8_t read(uint8_t address, uint8_t register_address, uint8_t & result) = 0;

private:
  AbstractI2C(const AbstractI2C & other);
  AbstractI2C & operator=(const AbstractI2C & rhs);
};

#endif
