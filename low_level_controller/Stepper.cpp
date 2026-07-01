#include "Stepper.h"

hw_timer_t* Stepper::leftTimer = nullptr;
hw_timer_t* Stepper::rightTimer = nullptr;

volatile bool Stepper::leftStepState = false;
volatile bool Stepper::rightStepState = false;

Stepper::Stepper() {
}

void Stepper::begin() {
    pinMode(Config::LEFT_STEP_PIN, OUTPUT);
    pinMode(Config::LEFT_DIR_PIN, OUTPUT);
    pinMode(Config::LEFT_EN_PIN, OUTPUT);

    pinMode(Config::RIGHT_STEP_PIN, OUTPUT);
    pinMode(Config::RIGHT_DIR_PIN, OUTPUT);
    pinMode(Config::RIGHT_EN_PIN, OUTPUT);

    digitalWrite(Config::LEFT_STEP_PIN, LOW);
    digitalWrite(Config::RIGHT_STEP_PIN, LOW);

    // Timer frequency = 1 MHz (1 tick = 1 us)
    leftTimer = timerBegin(1000000);
    rightTimer = timerBegin(1000000);

    timerAttachInterrupt(leftTimer, &Stepper::onLeftTimer);
    timerAttachInterrupt(rightTimer, &Stepper::onRightTimer);

    timerAlarm(
        leftTimer,
        Config::INITIAL_TIMER_PERIOD_US,
        true,
        0);

    timerAlarm(
        rightTimer,
        Config::INITIAL_TIMER_PERIOD_US,
        true,
        0);

    timerStop(leftTimer);
    timerStop(rightTimer);

    disable();
}


void Stepper::enable() {
    digitalWrite(
        Config::LEFT_EN_PIN,
        Config::DRIVER_ENABLE_LEVEL);

    digitalWrite(
        Config::RIGHT_EN_PIN,
        Config::DRIVER_ENABLE_LEVEL);

    enabled = true;
}

void Stepper::disable() {
    stop();

    digitalWrite(
        Config::LEFT_EN_PIN,
        Config::DRIVER_DISABLE_LEVEL);

    digitalWrite(
        Config::RIGHT_EN_PIN,
        Config::DRIVER_DISABLE_LEVEL);

    enabled = false;
}

bool Stepper::isEnabled() const {
    return enabled;
}

void Stepper::stop() {
    leftVelocityMps = 0.0f;
    rightVelocityMps = 0.0f;

    timerStop(leftTimer);
    timerStop(rightTimer);

    digitalWrite(Config::LEFT_STEP_PIN, LOW);
    digitalWrite(Config::RIGHT_STEP_PIN, LOW);

    leftStepState = false;
    rightStepState = false;
}

void Stepper::setMotion(float linearVelocityMps,
                        float angularVelocity) {
    if (!enabled) {
        return;
    }

    const float halfTrack =
        Config::TRACK_WIDTH_M * 0.5f;

    const float leftVelocity =
        linearVelocityMps - (angularVelocity * halfTrack);

    const float rightVelocity =
        linearVelocityMps + (angularVelocity * halfTrack);

    setLeftVelocity(leftVelocity);
    setRightVelocity(rightVelocity);
}

void Stepper::setLeftVelocity(float velocityMps) {
    leftVelocityMps = velocityMps;

    applyLeftStepRate(
        velocityToStepRate(velocityMps));
}

void Stepper::setRightVelocity(float velocityMps) {
    rightVelocityMps = velocityMps;

    applyRightStepRate(
        velocityToStepRate(velocityMps));
}

float Stepper::velocityToStepRate(
    float velocityMps) const {

    return velocityMps * Config::STEPS_PER_METER;
}

void Stepper::applyLeftStepRate(float stepRate) {
    if (fabsf(stepRate) < Config::MIN_STEP_RATE) {
        timerStop(leftTimer);
        digitalWrite(Config::LEFT_STEP_PIN, LOW);
        return;
    }

    digitalWrite(
        Config::LEFT_DIR_PIN,
        (stepRate >= 0.0f));

    stepRate = fabsf(stepRate);

    if (stepRate > Config::MAX_STEP_RATE) {
        stepRate = Config::MAX_STEP_RATE;
    }

    const uint32_t periodUs =
        static_cast<uint32_t>(
            1000000.0f / (2.0f * stepRate));

    timerAlarm(
        leftTimer,
        periodUs,
        true,
        0);

    timerStart(leftTimer);
}

void Stepper::applyRightStepRate(float stepRate) {
    if (fabsf(stepRate) < Config::MIN_STEP_RATE) {
        timerStop(rightTimer);
        digitalWrite(Config::RIGHT_STEP_PIN, LOW);
        return;
    }

    digitalWrite(
        Config::RIGHT_DIR_PIN,
        (stepRate >= 0.0f));

    stepRate = fabsf(stepRate);

    if (stepRate > Config::MAX_STEP_RATE) {
        stepRate = Config::MAX_STEP_RATE;
    }

    const uint32_t periodUs =
        static_cast<uint32_t>(
            1000000.0f / (2.0f * stepRate));

    timerAlarm(
        rightTimer,
        periodUs,
        true,
        0);

    timerStart(rightTimer);
}

void IRAM_ATTR Stepper::onLeftTimer() {
    leftStepState = !leftStepState;
    digitalWrite(
        Config::LEFT_STEP_PIN,
        leftStepState);
}

void IRAM_ATTR Stepper::onRightTimer() {
    rightStepState = !rightStepState;
    digitalWrite(
        Config::RIGHT_STEP_PIN,
        rightStepState);
}