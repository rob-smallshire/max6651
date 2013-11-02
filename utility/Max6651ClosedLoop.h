#ifndef __MAX6651CLOSEDLOOP_H__
#define __MAX6651CLOSEDLOOP_H__

#include <stdint.h>
#include <Arduino.h>

#include "Max6651Constants.h"

class ConfigureMax6651;
class AbstractI2C;

/**
 MAX6650 / MAX6651 configured as a closed-loop controller for fan speed.

 */
class Max6651ClosedLoop {
public:
  Max6651ClosedLoop();
  Max6651ClosedLoop(AbstractI2C* i2c, uint8_t address,
                    ConfigFanVoltage voltage = DEFAULT_FAN_VOLTAGE,
                    ConfigPrescaler prescaler = DEFAULT_PRESCALER,
                    TachometerCountTime tachometer_count_time = DEFAULT_TACHOMETER_COUNT_TIME,
                    AlarmEnable alarm_enable = DEFAULT_ALARM_ENABLE,
                    GpioDefinition gpio_definition = DEFAULT_GPIO_DEFINITION,
                    float clock_frequency = DEFAULT_CLOCK_FREQUENCY,
                    int pulses_per_revolution = DEFAULT_PULSES_PER_REVOLUTION,
                    Print* logger = 0);

  int begin(const ConfigureMax6651 & config);
  int begin();

  int stop(); // Store the current mode and go to full_off
  bool stopped();
  int run();  // Restore the current mode

  int targetSpeed(int rpm);
  int targetSpeed() const;

  int calibrateMinimumSpeed(uint8_t seconds, uint8_t index = 0);
  int calibrateMaximumSpeed(uint8_t seconds, uint8_t index = 0);

  int minimumSpeed() const;
  int maximumSpeed() const;
  int actualSpeed(uint8_t index = 0) const;

  int readAndResetAlarmStatus();

  int writeGpio(uint8_t index, bool state);
  bool readGpio(uint8_t index);

  ConfigPrescaler prescaler() const;
  int prescalerMultiplier() const;
  TachometerCountTime tachometerCountTime() const;
  float tachometerCountSeconds() const;
  float clockFrequencyHertz() const;

private:
  Max6651ClosedLoop(const Max6651ClosedLoop & other);
  Max6651ClosedLoop & operator=(const Max6651ClosedLoop & other);

  int setConfigRegister(uint8_t mode);
  int kTachToSpeed(uint8_t k_tach) const;
  uint8_t speedTokTach(int rpm);
  int setSpeed(uint8_t k_tach); // TODO: Better name
  int readTachometer(uint8_t index) const;

  uint8_t write(Register command, uint8_t data) const;
  uint8_t read(Register command, uint8_t & result) const;

  template <typename T>
  void print(T value) const {
    if (logger_) {
      logger_->print(value);
    }
  }

  template <typename T>
  void println(T value) const {
    if (logger_) {
      logger_->println(value);
    }
  }

  template <typename T>
  void printval(const char* name, T value) const {
      if (logger_) {
          logger_->print(name);
          logger_->print(" = ");
          logger_->println(value);
      }
  }

  AbstractI2C* i2c_;
  uint8_t address_;
  uint8_t mode_config_;
  uint8_t gpio_config_;
  uint8_t alarm_config_;
  uint8_t tachometer_count_time_;
  float f_clk_;
  Print* logger_;
  int target_rpm_;
  int pulses_per_revolution_;
  int k_tach_min_speed_;
  int k_tach_max_speed_;
};

#endif
