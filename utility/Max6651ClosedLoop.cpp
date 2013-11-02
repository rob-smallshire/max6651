#include <stdint.h>

#include "Max6651Constants.h"
#include "Max6651ClosedLoop.h"
#include "ConfigureMax6651.h"
#include "AbstractI2C.h"

static const float SECONDS_PER_MINUTE = 60.0;

Max6651ClosedLoop::Max6651ClosedLoop() :
    i2c_(0),
    address_(0),
    mode_config_(),
    gpio_config_(DEFAULT_GPIO_DEFINITION),
    alarm_config_(DEFAULT_ALARM_ENABLE),
    tachometer_count_time_(DEFAULT_TACHOMETER_COUNT_TIME),
    f_clk_(DEFAULT_CLOCK_FREQUENCY),
    logger_(0),
    target_rpm_(-1),
    pulses_per_revolution_(DEFAULT_PULSES_PER_REVOLUTION),
    k_tach_min_speed_(K_TACH_UPPER_LIMIT),
    k_tach_max_speed_(K_TACH_LOWER_LIMIT) {
}

Max6651ClosedLoop::Max6651ClosedLoop(AbstractI2C* i2c, uint8_t address,
                                      ConfigFanVoltage voltage,
                                      ConfigPrescaler prescaler,
                                     TachometerCountTime tachometer_count_time,
                                     AlarmEnable alarm_enable,
                                     GpioDefinition gpio_definition,
                                     float clock_frequency,
                                     int pulses_per_revolution,
                                     Print* logger) :
    i2c_(i2c),
    address_(address),
    mode_config_(),
    gpio_config_(gpio_definition),
    alarm_config_(alarm_enable),
    tachometer_count_time_(tachometer_count_time),
    f_clk_(clock_frequency),
    logger_(logger),
    target_rpm_(-1),
    pulses_per_revolution_(pulses_per_revolution),
    k_tach_min_speed_(K_TACH_UPPER_LIMIT),
    k_tach_max_speed_(K_TACH_LOWER_LIMIT) {
}

int Max6651ClosedLoop::begin(const ConfigureMax6651 & config) {
    i2c_ = config.i2c_;
    address_ = config.address_;
    mode_config_ = config.mode_config_;
    gpio_config_ = config.gpio_config_;
    alarm_config_ = config.alarm_config_;
    tachometer_count_time_ = config.tachometer_count_time_;
    f_clk_ = config.f_clk_;
    logger_ = config.logger_;
    pulses_per_revolution_ = config.pulses_per_revolution_;
    return begin();
}

int Max6651ClosedLoop::begin() {
    println("Max6651ClosedLoop::begin()");
    if (!i2c_) {
        //println("No I2C interface configured.");
        return -1;
    }
    int error = setConfigRegister(mode_config_);
    write(COMMAND_GPIO_DEFINE, gpio_config_);
    write(COMMAND_ALARM_ENABLE, alarm_config_);
    write(COMMAND_TACH_COUNT_TIME, tachometer_count_time_);
    targetSpeed(maximumSpeed());
    // TODO: Return error code
    return error;
}

/**
 *  Store the current mode, and set the controller
 *  to FULL_OFF_MODE
 */
int Max6651ClosedLoop::stop() {
  println("stop()");
  uint8_t off_mode = (mode_config_ & ~MODE_MASK) | MODE_FULL_OFF;
  return write(COMMAND_CONFIG, off_mode);
}

/**
 * Returns true if the controller is in FULL_OFF_MODE otherwise false;
 * Also returns true if communication with the device is unsuccessful.
 */
bool Max6651ClosedLoop::stopped() {
	uint8_t config;
	uint8_t error = read(COMMAND_CONFIG, config);
    if (error) {
	    return true;
	}
    return (config & MODE_MASK) == MODE_FULL_OFF;
}

int Max6651ClosedLoop::run() {
  println("run()");
  return write(COMMAND_CONFIG, mode_config_);
}

/**
 * Set the configuration register in the MAX6651.
 *
 * The 'mode' bits of the configuration byte will be ignored,
 * since they will always be set such that the mode is
 * MODE_CLOSED_LOOP.
 *
 * Args:
 *     config: The configuration byte.
 *
 * Returns:
 *     An error code. TODO
 */
int Max6651ClosedLoop::setConfigRegister(uint8_t config) {
    mode_config_ = MODE_CLOSED_LOOP | (config & ~MODE_MASK);
    return write(COMMAND_CONFIG, mode_config_);
}

/**
 * Set the target speed for the fan in RPM.
 *
 * Args:
 *     rpm: The desired speed in revolutions per minute.
 *
 * Returns:
 *     If successful, the *actual* target speed will be returned. This
 *     value may be different from the value passed in because of the
 *     limited precision and range of the MAX6651 registers. If
 *     unsuccessful a negative error code will be returned.
 */
int Max6651ClosedLoop::targetSpeed(int rpm) {
  if (rpm < 0) {
    //print("targetSpeed ");
    //print(rpm);
    //println(" out of range.");
    return -1;
  }

  if (rpm == 0) {
	  stop();
  }
  else {
	  if (stopped()) {
		  run();
	  }
  }

  // TODO: Validate against minimumSpeed() and maximumSpeed()

  uint8_t k_tach = speedTokTach(rpm);
  int result = setSpeed(k_tach);
  if (result < 0) {
      return result;
  }
  target_rpm_ = kTachToSpeed(k_tach);
  return target_rpm_;
}

/**
 * The RPM value set by the most recent successful call to targetSpeed(int).
 *
 * Returns:
 *
 */
int Max6651ClosedLoop::targetSpeed() const {
  return target_rpm_;
}

/**
 *  The minimum running speed in RPM which the device can set.
 *
 *  This call does not interact with the device. To set the
 *  device to the minimum speed pass the return value of this
 *  call to targetSpeed().
 *
 *  This speed may be below the stall speed of the motor, so
 *  though this is the minimum speed which the MAX6651 can
 *  attempt to regulate, there is no guarantee that the fan can
 *  run at this speed.
 *
 *  Returns:
 *      This minimum running speed in revolutions per minute which can
 *      be regulated by the MAX6651.
 */
int Max6651ClosedLoop::minimumSpeed() const {
  return kTachToSpeed(k_tach_min_speed_);
}

/**
 *  The maximum running speed in RPM which the device can set.
 *
 *  This call does not interact with the device. To set the
 *  device to the maximum speed pass the return value of this
 *  call to targetSpeed().
 *
 *  This speed may be above the top speed of the motor, so
 *  though this is the maximum speed which the MAX6651 can
 *  attempt to regulate, there is no guarantee that the fan can
 *  run at this speed.
 *
 *  Returns:
 *      This maximum running speed in revolutions per minute which can
 *      be regulated by the MAX6651.
 */
int Max6651ClosedLoop::maximumSpeed() const {
  return kTachToSpeed(k_tach_max_speed_);
}


int Max6651ClosedLoop::calibrateMinimumSpeed(uint8_t seconds, uint8_t index) {
  setSpeed(K_TACH_UPPER_LIMIT);
  unsigned long milliseconds = seconds * 1000UL;
  delay(milliseconds);
  int measured_rpm = actualSpeed(index);
  k_tach_min_speed_ = speedTokTach(measured_rpm);
  return kTachToSpeed(k_tach_min_speed_);
}

/**
 *
 */
int Max6651ClosedLoop::calibrateMaximumSpeed(uint8_t seconds, uint8_t index) {
  setSpeed(K_TACH_LOWER_LIMIT);
  unsigned long milliseconds = seconds * 1000UL;
  delay(milliseconds);
  int measured_rpm = actualSpeed(index);
  k_tach_max_speed_ = speedTokTach(measured_rpm);
  return kTachToSpeed(k_tach_max_speed_);
}

int Max6651ClosedLoop::kTachToSpeed(uint8_t k_tach) const {
  int prescaler_multiplier = prescalerMultiplier();
  float clock_frequency_hertz = clockFrequencyHertz();
  int k_tach_plus_one = int(k_tach) + 1;
  float numerator = prescaler_multiplier * clock_frequency_hertz;
  float denominator = (128.0f * pulses_per_revolution_ * k_tach_plus_one);
  float rps = numerator / denominator;
  int rpm = int(rps * SECONDS_PER_MINUTE);
  return rpm;
}

uint8_t Max6651ClosedLoop::speedTokTach(int rpm) {
    float rps = float(rpm) / SECONDS_PER_MINUTE;
    int k_tach = int((clockFrequencyHertz() * prescalerMultiplier()) / (128.0f * pulses_per_revolution_ * rps)) - 1;

    if (k_tach < 0) {
        k_tach = 0;
    }

    if (k_tach > 255) {
        k_tach = 255;
    }
    return uint8_t(k_tach);
}

int Max6651ClosedLoop::actualSpeed(uint8_t index) const {
  uint8_t tach = readTachometer(index); // 1
  float pulse_per_second = tach / tachometerCountSeconds(); // 0.5 = 1 / 2.0
  float rps = pulse_per_second / pulses_per_revolution_; // 0.25 = 0.5 / 2.0
  float rpm = rps * SECONDS_PER_MINUTE; // 15 = 0.25 * 60
  return int(rpm); // 15
}

int Max6651ClosedLoop::readTachometer(uint8_t index) const {
  if (index < 0 || index > 3) { // TODO: Fewer for MAX6650
     print("Invalid tachometer index ");
     println(index);
     return -1;
  }

  uint8_t result;
  uint8_t error = read(TACH_COMMAND[index], result);
  if (error) {
      return -error;
  }

  // Round a result of one down to zero. A stationary fan can
  // return 1, which is annoying.

  result = (result <= 1) ? 0 : result;

  return result;
}

int Max6651ClosedLoop::setSpeed(uint8_t k_tach) {
    return write(COMMAND_SPEED, k_tach);
}

int Max6651ClosedLoop::readAndResetAlarmStatus() {
  uint8_t result;
  uint8_t error = read(COMMAND_ALARM_STATUS, result);
  if (error) {
      return -error;
  }
  return result;
}

int Max6651ClosedLoop::writeGpio(uint8_t index, bool state) {
  uint8_t gpio_define;
  uint8_t error = read(COMMAND_GPIO_DEFINE, gpio_define);
  if (error) {
      return error;
  }
  gpio_define &= GPIO_DEFINE_MASK[index];
  gpio_define |= state ? GPIO_DEFINE_HIGH[index] : GPIO_DEFINE_LOW[index];
  return write(COMMAND_GPIO_DEFINE, gpio_define);
}

bool Max6651ClosedLoop::readGpio(uint8_t index) {
  uint8_t gpio_status;
  read(COMMAND_GPIO_STATUS, gpio_status);
  // TODO: Error handling
  return (gpio_status & GPIO_STATUS[index]) != 0;
}

ConfigPrescaler Max6651ClosedLoop::prescaler() const {
  return ConfigPrescaler(mode_config_ & PRESCALER_MASK);
}

int Max6651ClosedLoop::prescalerMultiplier() const{
    // 0 --> 1
    // 1 --> 2
    // 2 --> 4
    // 3 --> 8
    // 4 --> 16
    // n --> 2^n
    int n = int(prescaler());
    //return (n == 0) ? 0 : 1 << n;
    return (1 << (n+1)) >> 1;
}

TachometerCountTime Max6651ClosedLoop::tachometerCountTime() const {
  return TachometerCountTime(tachometer_count_time_);
}

float Max6651ClosedLoop::tachometerCountSeconds() const {
    // MAX6651 datasheet page 12, column 1
    return (1 << tachometer_count_time_) / 4.0f;
}

float Max6651ClosedLoop::clockFrequencyHertz() const {
  return f_clk_;
}


/**
 *  Write to the I2C device.
 *
 *  Returns: An error code.
 */
uint8_t Max6651ClosedLoop::write(Register command, uint8_t data) const {
  return i2c_->write(address_, uint8_t(command), data);
}

/**
 *  Read the specified register. If an error
 *  occurs, return error.
 *
 *  Returns a error code.
 */
uint8_t Max6651ClosedLoop::read(Register command, uint8_t & result) const {
  return i2c_->read(address_, uint8_t(command), result);
}

