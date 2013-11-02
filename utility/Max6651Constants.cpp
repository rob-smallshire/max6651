/*
 * Max6651Constants.cpp
 *
 *  Created on: Nov 4, 2011
 *      Author: rjs
 */

#include <stdint.h>
#include <Arduino.h>

#include "Max6651Constants.h"

const Register TACH_COMMAND[4] = { COMMAND_TACH_0,
                                   COMMAND_TACH_1,
                                   COMMAND_TACH_2,
                                   COMMAND_TACH_3 };

const uint8_t GPIO_DEFINE_MASK[5] = { B00000011,
                                      B00001100,
                                      B00110000,
                                      B01000000,
                                      B10000000 };

const GpioDefinition GPIO_DEFINE_HIGH[5] = { GPIO_DEFINE_0_HIGH_OR_INPUT,
                                             GPIO_DEFINE_1_HIGH_OR_INPUT,
                                             GPIO_DEFINE_2_HIGH_OR_INPUT,
                                             GPIO_DEFINE_3_HIGH_OR_INPUT,
                                             GPIO_DEFINE_4_HIGH_OR_INPUT };

const GpioDefinition (&GPIO_DEFINE_INPUT)[5] = GPIO_DEFINE_HIGH;

const GpioDefinition GPIO_DEFINE_LOW[5]  = { GPIO_DEFINE_0_LOW,
                                             GPIO_DEFINE_1_LOW,
                                             GPIO_DEFINE_2_LOW,
                                             GPIO_DEFINE_3_LOW,
                                             GPIO_DEFINE_4_LOW };

const GpioStatus GPIO_STATUS[5] = { GPIO_STATUS_0,
                                    GPIO_STATUS_1,
                                    GPIO_STATUS_2,
                                    GPIO_STATUS_3,
                                    GPIO_STATUS_4 };

const ConfigFanVoltage DEFAULT_FAN_VOLTAGE = VOLTAGE_12;
const ConfigPrescaler DEFAULT_PRESCALER = PRESCALER_DIVIDE_BY_4;
const TachometerCountTime DEFAULT_TACHOMETER_COUNT_TIME = TACH_COUNT_TIME_1_0;
const float DEFAULT_CLOCK_FREQUENCY = 254000.0f;
const uint8_t DEFAULT_PULSES_PER_REVOLUTION = 2;
const AlarmEnable DEFAULT_ALARM_ENABLE = AlarmEnable(0);
const GpioDefinition DEFAULT_GPIO_DEFINITION = GpioDefinition(0xff);
const uint8_t K_TACH_LOWER_LIMIT = 0;
const uint8_t K_TACH_UPPER_LIMIT = 255;

