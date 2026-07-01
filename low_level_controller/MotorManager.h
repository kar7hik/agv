#pragma once

#include <Arduino.h>
#include "Config.h"

class MotorManager {
public:
    MotorManager();

    void begin();

    void enableDrivers();
    void disableDrivers();

    void stopMotors();

    void setLeftFrequency(float freqHz);
    void setRightFrequency(float freqHz);
    void setFrequencies(float leftFreqHz, float rightFreqHz);

    float getLeftFrequency() const;
    float getRightFrequency() const;

    bool areDriversEnabled() const;

private:
    static void IRAM_ATTR onLeftTimer();
    static void IRAM_ATTR onRightTimer();

    void setupPins();
    void setupTimers();

    void applyLeftFrequency(float freqHz);
    void applyRightFrequency(float freqHz);

    float limitFrequency(float freqHz) const;
    bool isZeroFrequency(float freqHz) const;

private:
    static hw_timer_t* leftTimer;
    static hw_timer_t* rightTimer;

    static volatile bool leftStepState;
    static volatile bool rightStepState;

    float leftFrequencyHz;
    float rightFrequencyHz;

    bool driversEnabled;
};