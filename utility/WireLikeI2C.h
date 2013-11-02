#ifndef __WIRELIKEI2C_H__
#define __WIRELIKEI2C_H__

#include <stdint.h>

#include "AbstractI2C.h"

template <class T>
class WireLikeI2C : public AbstractI2C {
public:
  explicit WireLikeI2C(T & t);

  virtual uint8_t write(uint8_t address, uint8_t reg, uint8_t value);
  virtual uint8_t read(uint8_t address, uint8_t register_address, uint8_t & result);

private:
  WireLikeI2C(const WireLikeI2C & other);
  WireLikeI2C & operator=(const WireLikeI2C & rhs);
private:
  T* t_;
};

template <class T>
WireLikeI2C<T>::WireLikeI2C(T & t) :
  t_(&t)
{}

template <class T>
uint8_t WireLikeI2C<T>::write(uint8_t address, uint8_t register_address, uint8_t value) {
    t_->beginTransmission(address);
    t_->send(register_address);
    t_->send(value);
    return t_->endTransmission();
}

template <class T>
uint8_t WireLikeI2C<T>::read(uint8_t address, uint8_t register_address, uint8_t & result) {
    t_->beginTransmission(address);
    t_->send(register_address);
    t_->endTransmission();
    t_->requestFrom(address, uint8_t(1));
    if (t_->available() >= 1) {
      result = t_->receive();
      return 0;
    }
    return -1;
}

#endif
