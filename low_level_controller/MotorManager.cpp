#include "MotorManager.h"
#include <math.h>

hw_timer_t* MotorManager::_leftTimer = nullptr;
hw_timer_t* MotorManager::_rightTimer = nullptr;
volatile bool MotorManager::_leftStepState = false;
volatile bool MotorManager::_rightStepState = false;

void IRAM_ATTR MotorManager::onLeftTimer() {
    _leftStepState = !_leftStepState;
    digitalWrite(Config::LEFT_STEP_PIN, _leftStepState);
}

void IRAM_ATTR MotorManager::onRightTimer() {
    _rightStepState = !_rightStepState;
    digitalWrite(Config::RIGHT_STEP_PIN, _rightStepState);
}

bool MotorManager::begin() {
    pinMode(Config::LEFT_STEP_PIN, OUTPUT);
    pinMode(Config::LEFT_DIR_PIN, OUTPUT);
    pinMode(Config::LEFT_EN_PIN, OUTPUT);

    pinMode(Config::RIGHT_STEP_PIN, OUTPUT);
    pinMode(Config::RIGHT_DIR_PIN, OUTPUT);
    pinMode(Config::RIGHT_EN_PIN, OUTPUT);

    digitalWrite(Config::LEFT_STEP_PIN, LOW);
    digitalWrite(Config::RIGHT_STEP_PIN, LOW);
    disableDrivers();

    _leftTimer = timerBegin(Config::LEFT_TIMER_ID, Config::TIMER_PRESCALER, true);
    _rightTimer = timerBegin(Config::RIGHT_TIMER_ID, Config::TIMER_PRESCALER, true);

    if (_leftTimer == nullptr || _rightTimer == nullptr) {
        return false;
    }

    timerAttachInterrupt(_leftTimer, &MotorManager::onLeftTimer, true);
    timerAttachInterrupt(_rightTimer, &MotorManager::onRightTimer, true);

    timerAlarmWrite(_leftTimer, 1000, true);
    timerAlarmWrite(_rightTimer, 1000, true);
    timerAlarmDisable(_leftTimer);
    timerAlarmDisable(_rightTimer);

    return true;
}

void MotorManager::enableDrivers() {
    digitalWrite(Config::LEFT_EN_PIN, Config::DRIVER_ENABLE_LEVEL);
    digitalWrite(Config::RIGHT_EN_PIN, Config::DRIVER_ENABLE_LEVEL);
    _driversEnabled = true;
}

void MotorManager::disableDrivers() {
    stopMotors();
    digitalWrite(Config::LEFT_EN_PIN, Config::DRIVER_DISABLE_LEVEL);
    digitalWrite(Config::RIGHT_EN_PIN, Config::DRIVER_DISABLE_LEVEL);
    _driversEnabled = false;
}

bool MotorManager::driversEnabled() const {
    return _driversEnabled;
}

void MotorManager::setMotorFrequencies(float leftHz, float rightHz) {
    if (!_driversEnabled) {
        stopMotors();
        return;
    }

    leftHz = constrain(leftHz, -Config::MAX_MOTOR_FREQ_HZ, Config::MAX_MOTOR_FREQ_HZ);
    rightHz = constrain(rightHz, -Config::MAX_MOTOR_FREQ_HZ, Config::MAX_MOTOR_FREQ_HZ);

    setLeftFrequency(leftHz);
    setRightFrequency(rightHz);
}

void MotorManager::stopMotors() {
    stopLeftMotor();
    stopRightMotor();
}

float MotorManager::getLeftFrequencyHz() const {
    return _leftFrequencyHz;
}

float MotorManager::getRightFrequencyHz() const {
    return _rightFrequencyHz;
}

void MotorManager::setLeftFrequency(float frequencyHz) {
    if (fabsf(frequencyHz) < Config::MIN_EFFECTIVE_FREQ_HZ) {
        stopLeftMotor();
        return;
    }

    const bool forward = frequencyHz > 0.0f;
    digitalWrite(Config::LEFT_DIR_PIN, (forward ^ Config::LEFT_DIR_INVERTED) ? HIGH : LOW);

    const float absHz = fabsf(frequencyHz);
    const uint32_t halfPeriodUs = static_cast<uint32_t>(1000000.0f / (2.0f * absHz));

    timerAlarmWrite(_leftTimer, halfPeriodUs, true);
    timerAlarmEnable(_leftTimer);
    _leftFrequencyHz = frequencyHz;
}

void MotorManager::setRightFrequency(float frequencyHz) {
    if (fabsf(frequencyHz) < Config::MIN_EFFECTIVE_FREQ_HZ) {
        stopRightMotor();
        return;
    }

    const bool forward = frequencyHz > 0.0f;
    digitalWrite(Config::RIGHT_DIR_PIN, (forward ^ Config::RIGHT_DIR_INVERTED) ? HIGH : LOW);

    const float absHz = fabsf(frequencyHz);
    const uint32_t halfPeriodUs = static_cast<uint32_t>(1000000.0f / (2.0f * absHz));

    timerAlarmWrite(_rightTimer, halfPeriodUs, true);
    timerAlarmEnable(_rightTimer);
    _rightFrequencyHz = frequencyHz;
}

void MotorManager::stopLeftMotor() {
    if (_leftTimer != nullptr) {
        timerAlarmDisable(_leftTimer);
    }
    digitalWrite(Config::LEFT_STEP_PIN, LOW);
    _leftStepState = false;
    _leftFrequencyHz = 0.0f;
}

void MotorManager::stopRightMotor() {
    if (_rightTimer != nullptr) {
        timerAlarmDisable(_rightTimer);
    }
    digitalWrite(Config::RIGHT_STEP_PIN, LOW);
    _rightStepState = false;
    _rightFrequencyHz = 0.0f;
}
