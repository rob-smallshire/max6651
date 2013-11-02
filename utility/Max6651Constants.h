#ifndef __MAX6651_CONSTANTS_H__
#define __MAX6651_CONSTANTS_H__

#include <stdint.h>
#include <Arduino.h>

enum Register {
  COMMAND_SPEED           =  B00000000,
  COMMAND_CONFIG          =  B00000010,
  COMMAND_GPIO_DEFINE     =  B00000100,
  COMMAND_DAC             =  B00000110,
  COMMAND_ALARM_ENABLE    =  B00001000,
  COMMAND_ALARM_STATUS    =  B00001010,
  COMMAND_TACH_0          =  B00001100,
  COMMAND_TACH_1          =  B00001110,
  COMMAND_TACH_2          =  B00010000,
  COMMAND_TACH_3          =  B00010010,
  COMMAND_GPIO_STATUS     =  B00010100,
  COMMAND_TACH_COUNT_TIME =  B00010110
};

extern const Register TACH_COMMAND[4];

enum ConfigMask {
    MODE_MASK      = B00110000,
    VOLTAGE_MASK   = B00001000,
    PRESCALER_MASK = B00000111
};

enum ConfigMode {    
  MODE_FULL_ON     =      B00000000,
  MODE_FULL_OFF    =      B00010000,
  MODE_CLOSED_LOOP =      B00100000,
  MODE_OPEN_LOOP   =      B00110000
};

enum ConfigFanVoltage {
  VOLTAGE_12 = B00001000,
  VOLTAGE_5  = B00000000
};

extern const ConfigFanVoltage DEFAULT_FAN_VOLTAGE;

enum ConfigPrescaler {        // 76543210
  PRESCALER_DIVIDE_BY_1  = 0, // 00000000
  PRESCALER_DIVIDE_BY_2  = 1, // 00000001
  PRESCALER_DIVIDE_BY_4  = 2, // 00000010
  PRESCALER_DIVIDE_BY_8  = 3, // 00000011
  PRESCALER_DIVIDE_BY_16 = 4  // 00000100
};

extern const ConfigPrescaler DEFAULT_PRESCALER;

// GPIO Register

extern const uint8_t GPIO_DEFINE_MASK[];

enum GpioDefinition {                // 76543210
  GPIO_DEFINE_4_HIGH_OR_INPUT = 128, // 10000000
  GPIO_DEFINE_4_LOW           =   0, // 00000000
  
  GPIO_DEFINE_3_HIGH_OR_INPUT =  64, // 01000000
  GPIO_DEFINE_3_LOW           =   0, // 00000000
  
  GPIO_DEFINE_2_CLOCK_INPUT   =   0, // 00000000
  GPIO_DEFINE_2_CLOCK_OUTPUT  =  16, // 00010000
  GPIO_DEFINE_2_LOW           =  32, // 00100000
  GPIO_DEFINE_2_HIGH_OR_INPUT =  48, // 00110000
  
  GPIO_DEFINE_1_FULL_ON_INPUT =   4, // 00000100
  GPIO_DEFINE_1_LOW           =   8, // 00001000
  GPIO_DEFINE_1_HIGH_OR_INPUT =  12, // 00001100

  GPIO_DEFINE_0_ALERT_OUTPUT  =   1, // 00000001
  GPIO_DEFINE_0_LOW           =   2, // 00000010
  GPIO_DEFINE_0_HIGH_OR_INPUT =   3  // 00000011
};

extern const GpioDefinition DEFAULT_GPIO_DEFINITION;

extern const GpioDefinition GPIO_DEFINE_HIGH[5];
extern const GpioDefinition (&GPIO_DEFINE_INPUT)[5];
extern const GpioDefinition GPIO_DEFINE_LOW[5];

enum GpioStatus {            // 76543210
  GPIO_STATUS_4 = 16,        // 00010000
  GPIO_STATUS_3 =  8,        // 00001000
  GPIO_STATUS_2 =  4,        // 00000100
  GPIO_STATUS_1 =  2,        // 00000010
  GPIO_STATUS_0 =  1         // 00000001
};

extern const GpioStatus GPIO_STATUS[5];

enum AlarmEnable {                 // 76543210
  ALARM_ENABLE_GPIO_2        = 16, // 00010000
  ALARM_ENABLE_GPIO_1        =  8, // 00001000
  ALARM_ENABLE_TACH_OVERFLOW =  4, // 00000100
  ALARM_ENABLE_MIN_OUTPUT    =  2, // 00000010
  ALARM_ENABLE_MAX_OUTPUT    =  1, // 00000001
};

extern const AlarmEnable DEFAULT_ALARM_ENABLE;

enum AlarmStatus {                 // 76543210
  ALARM_STATUS_GPIO_2        = 16, // 00010000
  ALARM_STATUS_GPIO_1        =  8, // 00001000
  ALARM_STATUS_TACH_OVERFLOW =  4, // 00000100
  ALARM_STATUS_MIN_OUTPUT    =  2, // 00000010
  ALARM_STATUS_MAX_OUTPUT    =  1  // 00000001
};

enum TachometerCountTime {         // 76543210
  TACH_COUNT_TIME_0_25 = 0,        // 00000000
  TACH_COUNT_TIME_0_5  = 1,        // 00000001
  TACH_COUNT_TIME_1_0  = 2,        // 00000010
  TACH_COUNT_TIME_2_0  = 3         // 00000011
};

extern const TachometerCountTime DEFAULT_TACHOMETER_COUNT_TIME;

enum Address {
  ADDRESS_GND     = 72, // 1001000
  ADDRESS_VCC     = 75, // 1001011
  ADDRESS_HIGH_Z  = 27, // 0011011
  ADDRESS_10K_GND = 31  // 0011111
};

extern const float DEFAULT_CLOCK_FREQUENCY;
extern const uint8_t DEFAULT_PULSES_PER_REVOLUTION;
extern const uint8_t K_TACH_UPPER_LIMIT;
extern const uint8_t K_TACH_LOWER_LIMIT;

#endif
