#pragma once

#include <Arduino.h>
#include "Config.h"
#include "MotorManager.h"
#include "ImuManager.h"

enum class MotionMode : uint8_t {
    Idle = 0,
    NavigationControl = 1,
    ImuTurn = 2,
    Stopped = 3,
    Fault = 4
};

struct NavigationCommand {
    float velocityMps = 0.0f;
    float headingErrorDeg = 0.0f;
    float lateralErrorM = 0.0f;
    uint8_t flags = 0;
    uint32_t timestampMs = 0;
};

class MotionController {
public:
    bool begin(MotorManager* motorManager, ImuManager* imuManager);
    void update();

    void setNavigationCommand(const NavigationCommand& command);
    void stop();
    void enterIdle();
    void setFault();
    void clearFault();

    MotionMode getMode() const;
    NavigationCommand getNavigationCommand() const;
    float getBaseFrequencyHz() const;
    float getSteeringHz() const;

private:
    static float velocityMpsToStepHz(float velocityMps);

    MotorManager* _motorManager = nullptr;
    ImuManager* _imuManager = nullptr;

    MotionMode _mode = MotionMode::Idle;
    NavigationCommand _navCommand;

    uint32_t _lastCommandMillis = 0;
    uint32_t _lastControlMicros = 0;

    float _baseFrequencyHz = 0.0f;
    float _steeringHz = 0.0f;
};
