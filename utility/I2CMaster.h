#ifndef __I2CMASTER_H__
#define __I2CMASTER_H__

#include <stdint.h>

#include "AbstractI2C.h"

class I2C;

class I2CMaster : public AbstractI2C {
public:
  explicit I2CMaster(I2C & i2c);
  virtual ~I2CMaster() {}

  virtual uint8_t write(uint8_t address, uint8_t reg, uint8_t value);
  virtual uint8_t read(uint8_t address, uint8_t register_address, uint8_t & result);

private:
  I2CMaster(const I2CMaster & other);
  I2CMaster & operator=(const I2CMaster & rhs);

  I2C* t_;
};

#endif
