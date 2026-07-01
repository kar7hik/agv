#include "Arduino.h"
#include "HardwareSerial.h"
#include "Motion.h"

Motion::Motion(Stepper& stepper, Imu& imu)
    : stepper(stepper),
      imu(imu) {
}

void Motion::begin() {
    command = NavigationCommand();

    headingErrorDeg = 0.0f;
    angularVelocity = 0.0f;

    lastCommandTime = millis();
    lastUpdateTime = micros();

    stepper.stop();
}

void Motion::update() {
    const uint32_t now = micros();

    if ((now - lastUpdateTime) < Config::CONTROL_PERIOD_US) {
        return;
    }

    lastUpdateTime = now;

    if ((millis() - lastCommandTime) > Config::COMMAND_TIMEOUT_MS) {
        stop();
        return;
    }

    headingErrorDeg = normalizeAngle(
        command.desiredHeadingDeg - imu.getHeading());

    const float headingCorrection =
        Config::HEADING_KP * headingErrorDeg;

    const float headingCorrectionRad = headingCorrection * DEG_TO_RAD;

    const float lateralCorrection =
        Config::LATERAL_KP * command.lateralErrorM;

    angularVelocity =
        headingCorrectionRad + lateralCorrection;

    angularVelocity =
        limitAngularVelocity(angularVelocity);

    stepper.setMotion(
        command.velocityMps,
        angularVelocity);
}

void Motion::setNavigationCommand(
    const NavigationCommand& navigationCommand) {

    command = navigationCommand;

    command.velocityMps =
        limitVelocity(command.velocityMps);

    lastCommandTime = millis();
}

void Motion::stop() {
    command = NavigationCommand();

    headingErrorDeg = 0.0f;
    angularVelocity = 0.0f;

    stepper.stop();
}

NavigationCommand Motion::getNavigationCommand() const {
    return command;
}

float Motion::getHeadingError() const {
    Serial.println("headingErrorDeg: ");
    Serial.println(headingErrorDeg);
    return headingErrorDeg;
}

float Motion::normalizeAngle(float angleDeg) {
    while (angleDeg > Config::HEADING_MAX_DEG) {
        angleDeg -= Config::FULL_CIRCLE_DEG;
    }

    while (angleDeg < Config::HEADING_MIN_DEG) {
        angleDeg += Config::FULL_CIRCLE_DEG;
    }

    return angleDeg;
}

float Motion::limitVelocity(float velocityMps) const {
    if (velocityMps > Config::MAX_LINEAR_VELOCITY_MPS) {
        return Config::MAX_LINEAR_VELOCITY_MPS;
    }

    if (velocityMps < -Config::MAX_LINEAR_VELOCITY_MPS) {
        return -Config::MAX_LINEAR_VELOCITY_MPS;
    }

    return velocityMps;
}

float Motion::limitAngularVelocity(float angularVelocity) const {
    if (angularVelocity > Config::MAX_STEERING) {
        return Config::MAX_STEERING;
    }

    if (angularVelocity < -Config::MAX_STEERING) {
        return -Config::MAX_STEERING;
    }

    return angularVelocity;
}