/*
 * I2CMaster.cpp
 *
 *  Created on: Dec 13, 2011
 *      Author: rjs
 */
#include <stdint.h>
#include <Arduino.h>

#include <I2C.h>

#include "I2CMaster.h"

I2CMaster::I2CMaster(I2C & i2c) :
  t_(&i2c) {
}

uint8_t I2CMaster::write(uint8_t address, uint8_t register_address, uint8_t value) {
  Serial.print("I2CMaster::write(");
  Serial.print(int(address));
  Serial.print(", ");
  Serial.print(int(register_address));
  Serial.print(", ");
  Serial.print(int(value));
  Serial.print(") -> ");
  uint8_t error = t_->write(address, register_address, value);
  Serial.println(int(error));
  return error;
}

uint8_t I2CMaster::read(uint8_t address, uint8_t register_address, uint8_t & result) {
  Serial.print("I2CMaster:read(");
  Serial.print(int(address));
  Serial.print(", ");
  Serial.print(int(register_address));
  Serial.print(", ...) -> ");
  uint8_t error;
  error = t_->write(address, register_address);
  if (error) {
    Serial.print("write error =");
    Serial.println(int(error));
    return error;
  }

  error = t_->read(address, uint8_t(1));
  if (error) {
    Serial.print("read error =");
    Serial.println(int(error));
    return error;
  }

  if (t_->available() == 0) {
    return 1;
  }

  result = t_->receive();
  Serial.println(int(result));
  return 0;
}
