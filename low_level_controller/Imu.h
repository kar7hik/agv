#pragma once

#include <Arduino.h>
#include <Wire.h>


class Imu {
public:
    Imu();
    bool begin();
    bool calibrate();

    void update();

    void resetHeading();

    float getHeading() const;
    float getGyroRate() const;

    bool isCalibrated() const;

private:
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readGyroRaw(int16_t &raw);

    static float normalizeAngle(float angleDeg);

private:
    float headingDeg = 0.0f;
    float gyroRateDegPerSec = 0.0f;
    float gyroBiasDegPerSec = 0.0f;

    uint32_t lastUpdateMicros = 0;
    bool calibrated = false;
};