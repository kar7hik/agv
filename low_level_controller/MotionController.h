#pragma once

#include <Arduino.h>
#include <math.h>

#include "Config.h"
#include "MotorManager.h"
#include "ImuManager.h"

class MotionController {
public:
    enum class Mode : uint8_t {Stopped, VisualCorrection, ImuHeadingHold};

public:
    MotionController(MotorManager& motorManager, ImuManager& imuManager);

    void begin();
    void update();

    // Raspberry Pi sends only these three values.
    void setNavigationCommand(float velocityMps, float headingErrorDeg, float lateralErrorM);

    void stop();

    Mode getMode() const;

    float getCommandVelocityMps() const;

    float getVisualHeadingErrorDeg() const;
    float getVisualLateralErrorM() const;

    float getTargetHeadingDeg() const;
    float getImuHeadingErrorDeg() const;

    float getLeftFrequencyHz() const;
    float getRightFrequencyHz() const;

private:
    float limitVelocity(float velocityMps) const;
    float limitSteering(float steeringHz) const;

    float velocityToFrequency(float velocityMps) const;
    float normalizeAngle(float angleDeg) const;

    bool isVisualCorrectionFresh() const;

    float computeVisualSteeringHz() const;
    float computeImuHeadingHoldSteeringHz();

    void switchToImuHeadingHold();

private:
    MotorManager& motor;
    ImuManager& imu;

    Mode mode;

    float commandVelocityMps;

    float visualHeadingErrorDeg;
    float visualLateralErrorM;

    float targetHeadingDeg;
    float imuHeadingErrorDeg;

    float leftFrequencyHz;
    float rightFrequencyHz;

    uint32_t lastUpdateMicros;
    uint32_t lastVisualCorrectionMillis;
};