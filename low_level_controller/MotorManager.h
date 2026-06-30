#pragma once

#include <Arduino.h>
#include "Config.h"

class MotorManager {
public:
    bool begin();

    void enableDrivers();
    void disableDrivers();
    bool driversEnabled() const;

    void setMotorFrequencies(float leftHz, float rightHz);
    void stopMotors();

    float getLeftFrequencyHz() const;
    float getRightFrequencyHz() const;

private:
    static void IRAM_ATTR onLeftTimer();
    static void IRAM_ATTR onRightTimer();

    void setLeftFrequency(float frequencyHz);
    void setRightFrequency(float frequencyHz);
    void stopLeftMotor();
    void stopRightMotor();

    static hw_timer_t* _leftTimer;
    static hw_timer_t* _rightTimer;
    static volatile bool _leftStepState;
    static volatile bool _rightStepState;

    bool _driversEnabled = false;
    float _leftFrequencyHz = 0.0f;
    float _rightFrequencyHz = 0.0f;
};
