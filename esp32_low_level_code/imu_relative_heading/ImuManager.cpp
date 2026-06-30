#include "ImuManager.h"

#define MPU6050_ADDR 0x68

#define REG_PWR_MGMT_1 0x6B
#define REG_GYRO_CONFIG 0x1B
#define REG_GYRO_ZOUT_H 0x47

static constexpr float GYRO_SCALE_250DPS = 131.0f;

bool ImuManager::begin(){
    Wire.begin();

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x00);

    if (Wire.endTransmission() != 0){
        return false;
    }

    delay(100);

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_GYRO_CONFIG);
    Wire.write(0x00);   // ±250 deg/sec
    Wire.endTransmission();

    _lastUpdateMicros = micros();

    _initialized = true;

    return true;
}

bool ImuManager::calibrate(){
    if (!_initialized)
        return false;

    const int sampleCount = 1000;

    float gyroSum = 0.0f;

    Serial.println("IMU calibration started.");
    Serial.println("Keep AGV completely stationary.");

    for (int i = 0; i < sampleCount; i++){
        float gz;

        if (!readRawGyroZ(gz))
            return false;

        gyroSum += gz;

        delay(2);
    }

    _gyroBiasZDegPerSec = gyroSum / sampleCount;

    _relativeHeadingDeg = 0.0f;

    _lastUpdateMicros = micros();

    _calibrated = true;

    Serial.println("IMU calibration complete.");

    return true;
}

void ImuManager::update(){
    if (!_calibrated)
        return;

    uint32_t now = micros();

    float dt = (now - _lastUpdateMicros) / 1000000.0f;

    _lastUpdateMicros = now;

    float rawGyroZ;

    if (!readRawGyroZ(rawGyroZ))
        return;

    _gyroZDegPerSec = rawGyroZ - _gyroBiasZDegPerSec;

    _relativeHeadingDeg += _gyroZDegPerSec * dt;

    _relativeHeadingDeg = wrapAngleDeg(_relativeHeadingDeg);
}

float ImuManager::getHeadingDeg() const{
    return _relativeHeadingDeg;
}

float ImuManager::getGyroZDegPerSec() const{
    return _gyroZDegPerSec;
}

bool ImuManager::isInitialized() const{
    return _initialized;
}

bool ImuManager::isCalibrated() const{
    return _calibrated;
}

bool ImuManager::readRawGyroZ(float &gyroZDegPerSec){
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(REG_GYRO_ZOUT_H);

    if (Wire.endTransmission(false) != 0)
        return false;

    Wire.requestFrom(MPU6050_ADDR, 2);

    if (Wire.available() < 2)
        return false;

    int16_t raw =(Wire.read() << 8) | Wire.read();

    gyroZDegPerSec = raw / GYRO_SCALE_250DPS;

    return true;
}

float ImuManager::wrapAngleDeg(float angleDeg){
    while (angleDeg > 180.0f)
        angleDeg -= 360.0f;

    while (angleDeg < -180.0f)
        angleDeg += 360.0f;

    return angleDeg;
}