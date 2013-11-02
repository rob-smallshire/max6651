#include <stdint.h>
#include <Arduino.h>
#include <Print.h>

#include "ConfigureMax6651.h"
#include "AbstractI2C.h"

namespace {
  uint8_t setState(uint8_t value, bool state, uint8_t mask) {
    return state
      ? value | mask
      : value & ~mask;
  }
}

/*
 Establish by default the same settings as the MAX6650/MAX6651 have following
 Power-On-Reset: Full-on mode, 12 V fan, prescaler set to divide by 4 and
 all alarms disabled, a tachometer count time of one second and a clock frequency
 of 254 kHz.
 */
ConfigureMax6651::ConfigureMax6651(AbstractI2C & i2c, uint8_t address) :
  i2c_(&i2c),
  address_(address),
  mode_config_(MODE_FULL_ON | DEFAULT_FAN_VOLTAGE | DEFAULT_PRESCALER),
  alarm_config_(DEFAULT_ALARM_ENABLE),
  gpio_config_(DEFAULT_GPIO_DEFINITION),
  tachometer_count_time_(DEFAULT_TACHOMETER_COUNT_TIME),
  pulses_per_revolution_(DEFAULT_PULSES_PER_REVOLUTION),
  f_clk_(DEFAULT_CLOCK_FREQUENCY),
  logger_(0)
{
}

ConfigureMax6651 & ConfigureMax6651::voltage(ConfigFanVoltage v) {
  mode_config_ &= ~VOLTAGE_MASK;
  mode_config_ |= v;
  return *this;
}

ConfigureMax6651 & ConfigureMax6651::v5() {
  return voltage(VOLTAGE_5);
}

ConfigureMax6651 & ConfigureMax6651::v12() {
  return voltage(VOLTAGE_12);
}

ConfigureMax6651 & ConfigureMax6651::tachometerCountTime(TachometerCountTime time) {
  tachometer_count_time_ = time;
  return *this;
}

ConfigureMax6651 & ConfigureMax6651::gpioAsOutput(uint8_t index, bool high) {
  gpio_config_ &= ~GPIO_DEFINE_MASK[index];
  gpio_config_ |= high ? GPIO_DEFINE_HIGH[index] : GPIO_DEFINE_LOW[index];
  return *this;
}

ConfigureMax6651 & ConfigureMax6651::gpioAsInput(uint8_t index) {
  gpio_config_ &= ~GPIO_DEFINE_MASK[index];
  gpio_config_ |= GPIO_DEFINE_INPUT[index];  
  return *this;
}

ConfigureMax6651 & ConfigureMax6651::gpio2AsClockOutput() {
  gpio_config_ &= ~GPIO_DEFINE_MASK[2];
  gpio_config_ |= GPIO_DEFINE_2_CLOCK_OUTPUT;
  return *this;
}

ConfigureMax6651 & ConfigureMax6651::gpio2AsClockInput() {
  gpio_config_ &= ~GPIO_DEFINE_MASK[2];
  gpio_config_ |= GPIO_DEFINE_2_CLOCK_INPUT;
  return *this;
}

ConfigureMax6651 & ConfigureMax6651::gpio1AsFullOnInput() {
  gpio_config_ &= ~GPIO_DEFINE_MASK[1];
  gpio_config_ |= GPIO_DEFINE_1_FULL_ON_INPUT;
  return *this;
}

ConfigureMax6651 & ConfigureMax6651::gpio0AsAlertOutput() {
  gpio_config_ &= ~GPIO_DEFINE_MASK[0];
  gpio_config_ |= GPIO_DEFINE_0_ALERT_OUTPUT;
  return *this;
}


ConfigureMax6651 & ConfigureMax6651::preScaler(ConfigPrescaler p) {
  mode_config_ &= ~PRESCALER_MASK;
  mode_config_ |= p;
  return *this;
}


ConfigureMax6651 & ConfigureMax6651::clockFrequency(float hertz) {
  f_clk_ = hertz;
  return *this;
}


ConfigureMax6651 & ConfigureMax6651::maxAlarm(bool state) {
  alarm_config_ = setState(alarm_config_, state, ALARM_ENABLE_MAX_OUTPUT);
  return *this;
}

ConfigureMax6651 & ConfigureMax6651::minAlarm(bool state) {
  alarm_config_ = setState(alarm_config_, state, ALARM_ENABLE_MIN_OUTPUT);
  return *this;
}

ConfigureMax6651 & ConfigureMax6651::tachometerAlarm(bool state) {
  alarm_config_ = setState(alarm_config_, state, ALARM_ENABLE_TACH_OVERFLOW);
  return *this;
}

ConfigureMax6651 & ConfigureMax6651::gpio1Alarm(bool state) {
  alarm_config_ = setState(alarm_config_, state, ALARM_ENABLE_GPIO_1);
  return *this;
}

ConfigureMax6651 & ConfigureMax6651::gpio2Alarm(bool state) {
  alarm_config_ = setState(alarm_config_, state, ALARM_ENABLE_GPIO_2);
  return *this;
}

ConfigureMax6651 & ConfigureMax6651::logger(Print* logger) {
  logger_ = logger;
  return *this;
}
