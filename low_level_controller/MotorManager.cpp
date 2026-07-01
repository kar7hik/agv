#include "MotorManager.h"

hw_timer_t* MotorManager::leftTimer = nullptr;
hw_timer_t* MotorManager::rightTimer = nullptr;

volatile bool MotorManager::leftStepState = false;
volatile bool MotorManager::rightStepState = false;

MotorManager::MotorManager() : leftFrequencyHz(0.0f), rightFrequencyHz(0.0f), driversEnabled(false){}

void MotorManager::begin(){
    setupPins();
    setupTimers();
    disableDrivers();
    stopMotors();
}

void MotorManager::setupPins(){
    pinMode(Config::LEFT_STEP_PIN, OUTPUT);
    pinMode(Config::LEFT_DIR_PIN, OUTPUT);
    pinMode(Config::LEFT_EN_PIN, OUTPUT);

    pinMode(Config::RIGHT_STEP_PIN, OUTPUT);
    pinMode(Config::RIGHT_DIR_PIN, OUTPUT);
    pinMode(Config::RIGHT_EN_PIN, OUTPUT);

    digitalWrite(Config::LEFT_STEP_PIN, LOW);
    digitalWrite(Config::RIGHT_STEP_PIN, LOW);

    digitalWrite(Config::LEFT_DIR_PIN, LOW);
    digitalWrite(Config::RIGHT_DIR_PIN, LOW);

    digitalWrite(Config::LEFT_EN_PIN, Config::DRIVER_DISABLE_LEVEL);
    digitalWrite(Config::RIGHT_EN_PIN, Config::DRIVER_DISABLE_LEVEL);
}

void MotorManager::setupTimers(){
    leftTimer = timerBegin(Config::LEFT_TIMER_ID, Config::TIMER_PRESCALER, true);
    rightTimer = timerBegin(Config::RIGHT_TIMER_ID, Config::TIMER_PRESCALER, true);

    timerAttachInterrupt(leftTimer, &MotorManager::onLeftTimer, true);
    timerAttachInterrupt(rightTimer, &MotorManager::onRightTimer, true);

    timerAlarmWrite(leftTimer, Config::INITIAL_TIMER_ALARM_US, true);
    timerAlarmWrite(rightTimer, Config::INITIAL_TIMER_ALARM_US, true);

    timerAlarmDisable(leftTimer);
    timerAlarmDisable(rightTimer);
}

void MotorManager::enableDrivers(){
    digitalWrite(Config::LEFT_EN_PIN, Config::DRIVER_ENABLE_LEVEL);
    digitalWrite(Config::RIGHT_EN_PIN, Config::DRIVER_ENABLE_LEVEL);

    driversEnabled = true;
}

void MotorManager::disableDrivers(){
    stopMotors();

    digitalWrite(Config::LEFT_EN_PIN, Config::DRIVER_DISABLE_LEVEL);
    digitalWrite(Config::RIGHT_EN_PIN, Config::DRIVER_DISABLE_LEVEL);

    driversEnabled = false;
}

void MotorManager::stopMotors(){
    leftFrequencyHz = 0.0f;
    rightFrequencyHz = 0.0f;

    if (leftTimer != nullptr){
        timerAlarmDisable(leftTimer);
    }

    if (rightTimer != nullptr){
        timerAlarmDisable(rightTimer);
    }

    digitalWrite(Config::LEFT_STEP_PIN, LOW);
    digitalWrite(Config::RIGHT_STEP_PIN, LOW);

    leftStepState = false;
    rightStepState = false;
}

void MotorManager::setFrequencies(float leftFreqHz, float rightFreqHz){
    setLeftFrequency(leftFreqHz);
    setRightFrequency(rightFreqHz);
}

void MotorManager::setLeftFrequency(float freqHz){
    freqHz = limitFrequency(freqHz);
    leftFrequencyHz = freqHz;
    applyLeftFrequency(leftFrequencyHz);
}

void MotorManager::setRightFrequency(float freqHz){
    freqHz = limitFrequency(freqHz);
    rightFrequencyHz = freqHz;
    applyRightFrequency(rightFrequencyHz);
}

void MotorManager::applyLeftFrequency(float freqHz){
    if(leftTimer == nullptr){
        return;
    }

        if (isZeroFrequency(freqHz)) {
        timerAlarmDisable(leftTimer);
        digitalWrite(Config::LEFT_STEP_PIN, LOW);
        leftStepState = false;
        return;
    }

    if (freqHz >= 0.0f) {
        digitalWrite(Config::LEFT_DIR_PIN, HIGH);
    } else {
        digitalWrite(Config::LEFT_DIR_PIN, LOW);
        freqHz = -freqHz;
    }

    uint32_t alarmUs = static_cast<uint32_t>(1000000.0f / (2.0f * freqHz));

    if (alarmUs < 1){
        alarmUs = 1;
    }

    timerAlarmWrite(leftTimer, alarmUs, true);
    timerAlarmEnable(leftTimer);
}

void MotorManager::applyRightFrequency(float freqHz){
    if(rightTimer == nullptr){
        return;
    }

    if (isZeroFrequency(freqHz)) {
        timerAlarmDisable(rightTimer);
        digitalWrite(Config::RIGHT_STEP_PIN, LOW);
        rightStepState = false;
        return;
    }

    if (freqHz >= 0.0f) {
        digitalWrite(Config::RIGHT_DIR_PIN, HIGH);
    } else {
        digitalWrite(Config::RIGHT_DIR_PIN, LOW);
        freqHz = -freqHz;
    }

    uint32_t alarmUs = static_cast<uint32_t>(1000000.0f / (2.0f * freqHz));

    if (alarmUs < 1){
        alarmUs = 1;
    }

    timerAlarmWrite(rightTimer, alarmUs, true);
    timerAlarmEnable(rightTimer);
}

float MotorManager::limitFrequency(float freqHz) const{
    if (freqHz > Config::MAX_MOTOR_FREQ_HZ){
        return Config::MAX_MOTOR_FREQ_HZ;
    }

    if (freqHz < -Config::MAX_MOTOR_FREQ_HZ){
        return -Config::MAX_MOTOR_FREQ_HZ;
    }

    return freqHz;
}

bool MotorManager::isZeroFrequency(float freqHz) const{
    return fabs(freqHz) < Config::MIN_EFFECTIVE_FREQ_HZ;
}

float MotorManager::getLeftFrequency() const{
    return leftFrequencyHz;
}

float MotorManager::getRightFrequency() const{
    return rightFrequencyHz;
}

bool MotorManager::areDriversEnabled() const{
    return driversEnabled;
}

void IRAM_ATTR MotorManager::onLeftTimer(){
    leftStepState = !leftStepState;
    digitalWrite(Config::LEFT_STEP_PIN, leftStepState? HIGH : LOW);
}

void IRAM_ATTR MotorManager::onRightTimer(){
    rightStepState = !rightStepState;
    digitalWrite(Config::RIGHT_STEP_PIN, rightStepState? HIGH : LOW);
}
