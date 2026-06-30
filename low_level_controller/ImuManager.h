#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "Config.h"

class ImuManager {
public:
    bool begin();
    bool calibrate();
    void update();

    // Slow correction only. Do not use this as normal steering control.
    void applyHeadingCorrectionDeg(float correctionDeg);
    void resetHeading(float headingDeg = 0.0f);

    float getHeadingDeg() const;
    float getGyroZDegPerSec() const;
    float getGyroBiasZDegPerSec() const;

    bool isInitialized() const;
    bool isCalibrated() const;
    bool hasFault() const;

private:
    bool readRawGyroZ(float& gyroZDegPerSec);
    static float wrapAngleDeg(float angleDeg);

    bool _initialized = false;
    bool _calibrated = false;
    bool _fault = false;

    float _gyroBiasZDegPerSec = 0.0f;
    float _gyroZDegPerSec = 0.0f;
    float _relativeHeadingDeg = 0.0f;

    uint32_t _lastUpdateMicros = 0;
};
