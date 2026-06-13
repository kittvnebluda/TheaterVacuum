#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include "esp32-rmt-pwm-reader.h"
#include <Arduino.h>
#include <cstdint>

class RobotController {
public:
  /**
   * Constructor for differential drive robot controller.
   * Assumes 2-channel RC input: channel 0 (twist_y) for forward/back, channel 1
   * (twist_x) for turning. Motor 1 (left): EN1/IN1/IN2; Motor 2 (right):
   * EN2/IN3/IN4. PWM: 10-bit resolution, 1kHz frequency.
   * @param en1_pin PWM enable pin for left motor
   * @param in1_pin Direction pin 1 for left motor
   * @param in2_pin Direction pin 2 for left motor
   * @param en2_pin PWM enable pin for right motor
   * @param in3_pin Direction pin 1 for right motor
   * @param in4_pin Direction pin 2 for right motor
   * @param rc_ch0_pin GPIO pin for RC channel 0 (twist_y)
   * @param rc_ch1_pin GPIO pin for RC channel 1 (twist_x)
   */
  RobotController(int en1_pin, int in1_pin, int in2_pin, int en2_pin,
                  int in3_pin, int in4_pin, uint8_t rc_ch0_pin,
                  uint8_t rc_ch1_pin);

  /**
   * Initialize hardware: pin modes, default directions, LEDC PWM setup, RMT PWM
   * reader. Call once in setup().
   */
  void begin();

  /**
   * Update: Read RC PWM channels, normalize to speeds (-1023 to 1023), compute
   * differential drive, apply to motors. Handles signal loss (failsafe to
   * neutral). Call repeatedly in loop() for control.
   */
  void update();

private:
  // Motor pins
  int _en1_pin;
  int _in1_pin;
  int _in2_pin;
  int _en2_pin;
  int _in3_pin;
  int _in4_pin;

  // RC pins
  uint8_t _rc_pins[2];

  // PWM channels (hardcoded for simplicity)
  int _pwm_ch1; // Channel 0 for left motor
  int _pwm_ch2; // Channel 1 for right motor

  // PWM constants
  const int _pwm_res = 10;
  const int _pwm_freq = 1000;
  const int _max_duty = 1023;

  // RC normalization constants (typical servo PWM: 1000-2000µs, neutral 1500µs)
  const int _neutral = 1500;
  const int _deadzone_min = 1000;
  const int _deadzone_max = 2000;

  /**
   * Set speed for left motor (-1023 to 1023). Positive: forward, negative:
   * reverse, 0: stop.
   */
  void _setLeftMotor(int speed);

  /**
   * Set speed for right motor (-1023 to 1023). Positive: forward, negative:
   * reverse, 0: stop.
   */
  void _setRightMotor(int speed);

  /**
   * Normalize raw PWM µs value: failsafe to neutral if invalid or lost signal.
   */
  int _normalizeRaw(int raw);
};

#endif // ROBOTCONTROLLER_H
