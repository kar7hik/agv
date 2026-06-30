#include "ImuManager.h"
#include <math.h>

bool ImuManager::begin() {
    Wire.begin();

    Wire.beginTransmission(Config::MPU6050_ADDR);
    Wire.write(Config::REG_PWR_MGMT_1);
    Wire.write(0x00); // wake up
    if (Wire.endTransmission() != 0) {
        _fault = true;
        return false;
    }

    delay(100);

    Wire.beginTransmission(Config::MPU6050_ADDR);
    Wire.write(Config::REG_GYRO_CONFIG);
    Wire.write(0x00); // +/-250 deg/s
    if (Wire.endTransmission() != 0) {
        _fault = true;
        return false;
    }

    _lastUpdateMicros = micros();
    _initialized = true;
    _fault = false;
    return true;
}

bool ImuManager::calibrate() {
    if (!_initialized) {
        return false;
    }

    float gyroSum = 0.0f;

    for (uint16_t i = 0; i < Config::IMU_CALIBRATION_SAMPLES; ++i) {
        float gz = 0.0f;
        if (!readRawGyroZ(gz)) {
            _fault = true;
            return false;
        }
        gyroSum += gz;
        delay(Config::IMU_CALIBRATION_DELAY_MS);
    }

    _gyroBiasZDegPerSec = gyroSum / static_cast<float>(Config::IMU_CALIBRATION_SAMPLES);
    _gyroZDegPerSec = 0.0f;
    _relativeHeadingDeg = 0.0f;
    _lastUpdateMicros = micros();
    _calibrated = true;
    _fault = false;
    return true;
}

void ImuManager::update() {
    if (!_calibrated) {
        return;
    }

    const uint32_t now = micros();
    const float dt = static_cast<float>(now - _lastUpdateMicros) / 1000000.0f;
    _lastUpdateMicros = now;

    if (dt <= 0.0f || dt > 0.25f) {
        return;
    }

    float rawGyroZ = 0.0f;
    if (!readRawGyroZ(rawGyroZ)) {
        _fault = true;
        return;
    }

    _gyroZDegPerSec = rawGyroZ - _gyroBiasZDegPerSec;
    _relativeHeadingDeg = wrapAngleDeg(_relativeHeadingDeg + (_gyroZDegPerSec * dt));
}

void ImuManager::applyHeadingCorrectionDeg(float correctionDeg) {
    if (!_calibrated) {
        return;
    }
    const float boundedCorrection = constrain(correctionDeg, -15.0f, 15.0f);
    _relativeHeadingDeg = wrapAngleDeg(_relativeHeadingDeg + boundedCorrection * Config::IMU_HEADING_CORRECTION_ALPHA);
}

void ImuManager::resetHeading(float headingDeg) {
    _relativeHeadingDeg = wrapAngleDeg(headingDeg);
    _lastUpdateMicros = micros();
}

float ImuManager::getHeadingDeg() const {
    return _relativeHeadingDeg;
}

float ImuManager::getGyroZDegPerSec() const {
    return _gyroZDegPerSec;
}

float ImuManager::getGyroBiasZDegPerSec() const {
    return _gyroBiasZDegPerSec;
}

bool ImuManager::isInitialized() const {
    return _initialized;
}

bool ImuManager::isCalibrated() const {
    return _calibrated;
}

bool ImuManager::hasFault() const {
    return _fault;
}

bool ImuManager::readRawGyroZ(float& gyroZDegPerSec) {
    Wire.beginTransmission(Config::MPU6050_ADDR);
    Wire.write(Config::REG_GYRO_ZOUT_H);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    const uint8_t bytesRequested = 2;
    if (Wire.requestFrom(Config::MPU6050_ADDR, bytesRequested) != bytesRequested) {
        return false;
    }

    const int16_t raw = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
    gyroZDegPerSec = static_cast<float>(raw) / Config::GYRO_SCALE_250DPS;
    return true;
}

float ImuManager::wrapAngleDeg(float angleDeg) {
    while (angleDeg > 180.0f) {
        angleDeg -= 360.0f;
    }
    while (angleDeg < -180.0f) {
        angleDeg += 360.0f;
    }
    return angleDeg;
}
