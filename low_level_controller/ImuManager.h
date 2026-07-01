#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "Config.h"

class ImuManager {
public:
    ImuManager();

    bool begin();

    void calibrate();
    void update();

    float getHeadingDeg() const;
    float getGyroZDegPerSec() const;
    float getGyroZBias() const;

    bool isCalibrated() const;

    void resetHeading();
    void setHeading(float headingDeg);

private:
    void writeRegister(uint8_t reg, uint8_t value);
    int16_t readGyroZRaw();

    float normalizeHeading(float headingDeg) const;

private:
    float headingDeg;
    float gyroZDegPerSec;
    float gyroZBias;

    uint32_t lastUpdateMicros;

    bool calibrated;
};