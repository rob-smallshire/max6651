#ifndef __CONFIGURE_MAX6651_H__
#define __CONFIGURE_MAX6651_H__

#include <stdint.h>
#include <Arduino.h>

#include "Max6651Constants.h"

class Print;

class AbstractI2C; 

/*
 
 For example, to use with the popular Wire library:

 // Create an adapter to the Wire library
 typedef WireLikeI2C<TwoWire> WireI2C;
 WireI2C wire_i2c(Wire);

 // Create a default configured controller
 Max6651Base controller = ConfigureMax6651(wire_i2c, ADDRESS_GND);
 
 Max6651ClosedLoop controller = ConfigureMax6651(wire_i2c, ADDRESS_10K_GND)
                                  .v5()
                                  .preScaler(4)
                                  .clockFrequency(400)
;                                 .tachometerCountTime(2)
                                  .tachometerAlarm()
                                  .minAlarm(false)
                                  .maxAlarm(false);

 */
struct ConfigureMax6651 {
  ConfigureMax6651(AbstractI2C & i2c, uint8_t address);

  ConfigureMax6651 & voltage(ConfigFanVoltage v);
  ConfigureMax6651 & v5();
  ConfigureMax6651 & v12();

  ConfigureMax6651 & preScaler(ConfigPrescaler p);
  ConfigureMax6651 & clockFrequency(float hertz);
  ConfigureMax6651 & tachometerCountTime(TachometerCountTime time);

  ConfigureMax6651 & gpioAsOutput(uint8_t index, bool high);
  ConfigureMax6651 & gpioAsInput(uint8_t index);

  ConfigureMax6651 & gpio2AsClockOutput();
  ConfigureMax6651 & gpio2AsClockInput();

  ConfigureMax6651 & gpio1AsFullOnInput();

  ConfigureMax6651 & gpio0AsAlertOutput();

  // TODO: Put alarm first in the method names
  ConfigureMax6651 & enableAlarms(AlarmEnable a);
  ConfigureMax6651 & maxAlarm(bool enable = true);
  ConfigureMax6651 & minAlarm(bool enable = true);
  ConfigureMax6651 & tachometerAlarm(bool enable = true);
  ConfigureMax6651 & gpio1Alarm(bool state = true);
  ConfigureMax6651 & gpio2Alarm(bool state = true);

  ConfigureMax6651 & logger(Print* l);

  AbstractI2C* i2c_;
  uint8_t address_;
  uint8_t mode_config_;
  uint8_t alarm_config_;
  uint8_t gpio_config_;
  uint8_t tachometer_count_time_;
  uint8_t pulses_per_revolution_;
  float f_clk_;
  Print* logger_;
};

#endif
