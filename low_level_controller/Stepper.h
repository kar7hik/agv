#pragma once

#include <Arduino.h>

#include "Config.h"

class Stepper {
public:
    Stepper();

    void begin();

    void enable();
    void disable();

    bool isEnabled() const;

    void stop();

    void setMotion(float linearVelocityMps,
                   float angularVelocity);

private:
    void setLeftVelocity(float velocityMps);
    void setRightVelocity(float velocityMps);

    void applyLeftStepRate(float stepRate);
    void applyRightStepRate(float stepRate);

    float velocityToStepRate(float velocityMps) const;

    static void IRAM_ATTR onLeftTimer();
    static void IRAM_ATTR onRightTimer();

private:
    static hw_timer_t* leftTimer;
    static hw_timer_t* rightTimer;

    static volatile bool leftStepState;
    static volatile bool rightStepState;

    bool enabled = false;

    float leftVelocityMps = 0.0f;
    float rightVelocityMps = 0.0f;
};