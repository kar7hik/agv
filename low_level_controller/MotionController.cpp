#include "MotionController.h"

MotionController::MotionController(MotorManager& motorManager, ImuManager& imuManager):
    motor(motorManager),imu(imuManager), mode(Mode::Stopped), commandVelocityMps(0.0f),
    visualHeadingErrorDeg(0.0f), visualLateralErrorM(0.0f), targetHeadingDeg(0.0f),
    imuHeadingErrorDeg(0.0f), leftFrequencyHz(0.0f), rightFrequencyHz(0.0f),
    lastUpdateMicros(0), lastVisualCorrectionMillis(0){}

void MotionController::begin(){
    mode = Mode::Stopped;

    commandVelocityMps = 0.0f;

    visualHeadingErrorDeg = 0.0f;
    visualLateralErrorM = 0.0f;

    targetHeadingDeg = imu.getHeadingDeg();
    imuHeadingErrorDeg = 0.0f;

    leftFrequencyHz = 0.0f;
    rightFrequencyHz = 0.0f;

    lastUpdateMicros = micros();
    lastVisualCorrectionMillis = millis();

    motor.stopMotors();
}

void MotionController::setNavigationCommand(float velocityMps, float headingErrorDeg, float lateralErrorM){
    commandVelocityMps = limitVelocity(velocityMps);

    visualHeadingErrorDeg = headingErrorDeg;
    visualLateralErrorM = lateralErrorM;

    lastVisualCorrectionMillis = millis();

    // Every packet from Pi is treated as fresh visual correction.
    // Correction will be used only until VISUAL_CORRECTION_TIMEOUT_MS expires.
    mode = Mode::VisualCorrection;
}

void MotionController::update(){
    uint32_t now = micros();

    if ((now - lastUpdateMicros) < Config::CONTROL_PERIOD_US) {
        return;
    }

    lastUpdateMicros = now;

    if (mode == Mode::Stopped) {
        motor.stopMotors();
        return;
    }

    float baseFreqHz = velocityToFrequency(commandVelocityMps);
    float steeringHz = 0.0f;

    if (mode == Mode::VisualCorrection) {
        if (isVisualCorrectionFresh()) {
            steeringHz = computeVisualSteeringHz();
        } else {
            switchToImuHeadingHold();
            steeringHz = computeImuHeadingHoldSteeringHz();
        }
    } else if (mode == Mode::ImuHeadingHold) {
        steeringHz = computeImuHeadingHoldSteeringHz();
    }

    steeringHz = limitSteering(steeringHz);

    leftFrequencyHz = baseFreqHz - steeringHz;
    rightFrequencyHz = baseFreqHz + steeringHz;

    motor.setFrequencies(leftFrequencyHz, rightFrequencyHz);
}

void MotionController::stop(){
    mode = Mode::Stopped;

    commandVelocityMps = 0.0f;

    visualHeadingErrorDeg = 0.0f;
    visualLateralErrorM = 0.0f;

    imuHeadingErrorDeg = 0.0f;

    leftFrequencyHz = 0.0f;
    rightFrequencyHz = 0.0f;

    motor.stopMotors();
}

void MotionController::switchToImuHeadingHold(){
    /*
        Locked rule:

        AprilTag heading is truth while visible.
        IMU is only used to hold direction between tags.

        When visual correction expires, do not force IMU heading to zero.
        Whatever IMU reads at that moment becomes the new straight reference.
    */
    targetHeadingDeg = imu.getHeadingDeg();

    visualHeadingErrorDeg = 0.0f;
    visualLateralErrorM = 0.0f;

    imuHeadingErrorDeg = 0.0f;

    mode = Mode::ImuHeadingHold;
}

bool MotionController::isVisualCorrectionFresh() const{
    return (millis() - lastVisualCorrectionMillis) <= Config::VISUAL_CORRECTION_TIMEOUT_MS;
}

float MotionController::computeVisualSteeringHz() const{
    float steeringHz = Config::STEERING_DIRECTION *(Config::HEADING_KP_HZ_PER_DEG * visualHeadingErrorDeg + Config::LATERAL_KP_HZ_PER_M * visualLateralErrorM);

    return steeringHz;
}

float MotionController::computeImuHeadingHoldSteeringHz(){
    imuHeadingErrorDeg = normalizeAngle(targetHeadingDeg - imu.getHeadingDeg());

    float steeringHz = Config::STEERING_DIRECTION *(Config::HEADING_KP_HZ_PER_DEG * imuHeadingErrorDeg);

    return steeringHz;
}

float MotionController::limitVelocity(float velocityMps) const{
    if (velocityMps > Config::MAX_VELOCITY_MPS) {
        return Config::MAX_VELOCITY_MPS;
    }

    if (velocityMps < -Config::MAX_VELOCITY_MPS) {
        return -Config::MAX_VELOCITY_MPS;
    }

    return velocityMps;
}

float MotionController::limitSteering(float steeringHz) const{
    if (steeringHz > Config::MAX_STEERING_HZ) {
        return Config::MAX_STEERING_HZ;
    }

    if (steeringHz < -Config::MAX_STEERING_HZ) {
        return -Config::MAX_STEERING_HZ;
    }

    return steeringHz;
}

float MotionController::velocityToFrequency(float velocityMps) const{
    return velocityMps * Config::PULSES_PER_METER;
}

float MotionController::normalizeAngle(float angleDeg) const{
    while (angleDeg > Config::HEADING_MAX_DEG) {
        angleDeg -= Config::FULL_ROTATION_DEG;
    }

    while (angleDeg < Config::HEADING_MIN_DEG) {
        angleDeg += Config::FULL_ROTATION_DEG;
    }

    return angleDeg;
}

MotionController::Mode MotionController::getMode() const{
    return mode;
}

float MotionController::getCommandVelocityMps() const{
    return commandVelocityMps;
}

float MotionController::getVisualHeadingErrorDeg() const{
    return visualHeadingErrorDeg;
}

float MotionController::getVisualLateralErrorM() const{
    return visualLateralErrorM;
}

float MotionController::getTargetHeadingDeg() const{
    return targetHeadingDeg;
}

float MotionController::getImuHeadingErrorDeg() const{
    return imuHeadingErrorDeg;
}

float MotionController::getLeftFrequencyHz() const{
    return leftFrequencyHz;
}

float MotionController::getRightFrequencyHz() const{
    return rightFrequencyHz;
}