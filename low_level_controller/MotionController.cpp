#include "MotionController.h"
#include <math.h>

namespace {
constexpr uint8_t NAV_FLAG_HEADING_VALID = 1 << 0;
constexpr uint8_t NAV_FLAG_LATERAL_VALID = 1 << 1;
constexpr uint8_t NAV_FLAG_ESTOP = 1 << 2;
}

bool MotionController::begin(MotorManager* motorManager, ImuManager* imuManager) {
    if (motorManager == nullptr || imuManager == nullptr) {
        return false;
    }

    _motorManager = motorManager;
    _imuManager = imuManager;
    _mode = MotionMode::Idle;
    _lastCommandMillis = millis();
    _lastControlMicros = micros();
    return true;
}

void MotionController::update() {
    if (_motorManager == nullptr) {
        return;
    }

    const uint32_t nowUs = micros();
    if ((nowUs - _lastControlMicros) < Config::CONTROL_PERIOD_US) {
        return;
    }
    _lastControlMicros = nowUs;

    if (_mode == MotionMode::Fault || _mode == MotionMode::Stopped || _mode == MotionMode::Idle) {
        _motorManager->stopMotors();
        return;
    }

    if ((millis() - _lastCommandMillis) > Config::COMMAND_TIMEOUT_MS) {
        stop();
        return;
    }

    if ((_navCommand.flags & NAV_FLAG_ESTOP) != 0) {
        stop();
        return;
    }

    const float velocity = constrain(_navCommand.velocityMps,
                                     -Config::MAX_VELOCITY_MPS,
                                      Config::MAX_VELOCITY_MPS);

    _baseFrequencyHz = velocityMpsToStepHz(velocity);

    const float headingTerm = ((_navCommand.flags & NAV_FLAG_HEADING_VALID) != 0)
        ? Config::HEADING_KP_HZ_PER_DEG * _navCommand.headingErrorDeg
        : 0.0f;

    const float lateralTerm = ((_navCommand.flags & NAV_FLAG_LATERAL_VALID) != 0)
        ? Config::LATERAL_KP_HZ_PER_M * _navCommand.lateralErrorM
        : 0.0f;

    _steeringHz = Config::STEERING_DIRECTION * constrain(headingTerm + lateralTerm,
                                                         -Config::MAX_STEERING_HZ,
                                                          Config::MAX_STEERING_HZ);

    const float leftHz = _baseFrequencyHz - _steeringHz;
    const float rightHz = _baseFrequencyHz + _steeringHz;

    _motorManager->setMotorFrequencies(leftHz, rightHz);
}

void MotionController::setNavigationCommand(const NavigationCommand& command) {
    _navCommand = command;
    _lastCommandMillis = millis();

    if (_mode != MotionMode::Fault && _mode != MotionMode::Stopped) {
        _mode = MotionMode::NavigationControl;
    }
}

void MotionController::stop() {
    _mode = MotionMode::Stopped;
    _baseFrequencyHz = 0.0f;
    _steeringHz = 0.0f;
    if (_motorManager != nullptr) {
        _motorManager->stopMotors();
    }
}

void MotionController::enterIdle() {
    _mode = MotionMode::Idle;
    if (_motorManager != nullptr) {
        _motorManager->stopMotors();
    }
}

void MotionController::setFault() {
    _mode = MotionMode::Fault;
    if (_motorManager != nullptr) {
        _motorManager->stopMotors();
    }
}

void MotionController::clearFault() {
    if (_mode == MotionMode::Fault) {
        _mode = MotionMode::Idle;
    }
}

MotionMode MotionController::getMode() const {
    return _mode;
}

NavigationCommand MotionController::getNavigationCommand() const {
    return _navCommand;
}

float MotionController::getBaseFrequencyHz() const {
    return _baseFrequencyHz;
}

float MotionController::getSteeringHz() const {
    return _steeringHz;
}

float MotionController::velocityMpsToStepHz(float velocityMps) {
    return velocityMps * Config::STEPS_PER_METER;
}
