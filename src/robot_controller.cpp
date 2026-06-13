#include "robot_controller.hpp"
#include "esp32-hal-ledc.h"

RobotController::RobotController(int en1_pin, int in1_pin, int in2_pin, int en2_pin, int in3_pin, int in4_pin,
                                 uint8_t rc_ch0_pin, uint8_t rc_ch1_pin)
    : _en1_pin(en1_pin), _in1_pin(in1_pin), _in2_pin(in2_pin), _en2_pin(en2_pin), _in3_pin(in3_pin), _in4_pin(in4_pin) {
    _rc_pins[0] = rc_ch0_pin; // Channel 0: twist_y (forward/back)
    _rc_pins[1] = rc_ch1_pin; // Channel 1: twist_x (turn)
    _pwm_ch1 = 0;
    _pwm_ch2 = 1;
}

void RobotController::begin() {
    // Setup motor direction pins (default: forward)
    pinMode(_in1_pin, OUTPUT);
    pinMode(_in2_pin, OUTPUT);
    pinMode(_in3_pin, OUTPUT);
    pinMode(_in4_pin, OUTPUT);
    digitalWrite(_in1_pin, HIGH);
    digitalWrite(_in2_pin, LOW);
    digitalWrite(_in3_pin, HIGH);
    digitalWrite(_in4_pin, LOW);

    // Setup PWM (LEDC)
    ledcAttachPin(_en1_pin, _pwm_ch1);
    ledcAttachPin(_en2_pin, _pwm_ch2);
    ledcSetup(_pwm_ch1, _pwm_freq, _pwm_res);
    ledcSetup(_pwm_ch2, _pwm_freq, _pwm_res);

    // Setup RMT PWM reader
    pwm_reader_init(_rc_pins, 2);
    esp_err_t err = pwm_reader_begin();
    if (err != ESP_OK) {
        Serial.printf("PWM reader begin() error: %d\n", err);
    }
}

void RobotController::update() {
    int raw_y = pwm_get_rawPwm(0);
    int raw_x = pwm_get_rawPwm(1);

    raw_x = _normalizeRaw(raw_x);
    raw_y = _normalizeRaw(raw_y);

    // Normalize to -1.0 to 1.0
    float fwd = (static_cast<float>(raw_x) - _neutral) / 500.0f;
    float trn = (static_cast<float>(raw_y) - _neutral) / 500.0f;

    // Differential drive: left = fwd + trn, right = fwd - trn
    int left_speed = constrain(static_cast<int>((fwd - trn) * _max_duty), -_max_duty, _max_duty);
    int right_speed = constrain(static_cast<int>((fwd + trn) * _max_duty), -_max_duty, _max_duty);

    _setLeftMotor(left_speed);
    _setRightMotor(right_speed);

    // Debug output
    Serial.printf("Raw X:%d Y:%d Fwd:%.2f Trn:%.2f Left:%d Right:%d\n", raw_x, raw_y, fwd, trn, left_speed,
                  right_speed);
}

void RobotController::_setLeftMotor(int speed) {
    int duty = abs(speed);
    if (speed >= 0) {
        // Forward
        digitalWrite(_in1_pin, HIGH);
        digitalWrite(_in2_pin, LOW);
    } else {
        // Reverse
        digitalWrite(_in1_pin, LOW);
        digitalWrite(_in2_pin, HIGH);
    }
    ledcWrite(_pwm_ch1, duty);
}

void RobotController::_setRightMotor(int speed) {
    int duty = abs(speed);
    if (speed >= 0) {
        // Forward
        digitalWrite(_in3_pin, HIGH);
        digitalWrite(_in4_pin, LOW);
    } else {
        // Reverse
        digitalWrite(_in3_pin, LOW);
        digitalWrite(_in4_pin, HIGH);
    }
    ledcWrite(_pwm_ch2, duty);
}

int RobotController::_normalizeRaw(int raw) {
    if (raw == 0 || raw < _deadzone_min || raw > _deadzone_max) {
        return _neutral; // Failsafe to neutral
    }
    return raw;
}
