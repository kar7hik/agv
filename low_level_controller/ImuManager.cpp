#include "ImuManager.h"

ImuManager::ImuManager(): headingDeg(0.0f),gyroZDegPerSec(0.0f),
      gyroZBias(0.0f),lastUpdateMicros(0),calibrated(false){}

bool ImuManager::begin(){
    Wire.begin();

    writeRegister(Config::REG_PWR_MGMT_1, 0x00);
    delay(100);

    // Gyro full scale range = +/-250 deg/s.
    writeRegister(Config::REG_GYRO_CONFIG, 0x00);
    delay(50);

    lastUpdateMicros = micros();

    return true;
}

void ImuManager::calibrate(){
    long sum = 0;

    for (uint16_t i = 0; i < Config::IMU_CALIBRATION_SAMPLES; i++) {
        sum += readGyroZRaw();
        delay(Config::IMU_CALIBRATION_DELAY_MS);
    }

    float averageRaw = static_cast<float>(sum) / Config::IMU_CALIBRATION_SAMPLES;

    gyroZBias = averageRaw / Config::GYRO_SCALE_250DPS;

    headingDeg = 0.0f;
    gyroZDegPerSec = 0.0f;
    lastUpdateMicros = micros();
    calibrated = true;
}

//   Clockwise rotation         = negative heading
//   Counter-clockwise rotation = positive heading
void ImuManager::update(){
    if (!calibrated) {
        return;
    }

    uint32_t now = micros();
    float dtSec = (now - lastUpdateMicros) / 1000000.0f;
    lastUpdateMicros = now;

    int16_t rawGyroZ = readGyroZRaw();

    gyroZDegPerSec = (static_cast<float>(rawGyroZ) / Config::GYRO_SCALE_250DPS) - gyroZBias;

    headingDeg += gyroZDegPerSec * dtSec;
    headingDeg = normalizeHeading(headingDeg);
}

float ImuManager::getHeadingDeg() const{
    return headingDeg;
}

float ImuManager::getGyroZDegPerSec() const{
    return gyroZDegPerSec;
}

float ImuManager::getGyroZBias() const{
    return gyroZBias;
}

bool ImuManager::isCalibrated() const{
    return calibrated;
}

void ImuManager::resetHeading(){
    headingDeg = 0.0f;
    lastUpdateMicros = micros();
}

void ImuManager::setHeading(float headingDegValue){
    headingDeg = normalizeHeading(headingDegValue);
    lastUpdateMicros = micros();
}

void ImuManager::writeRegister(uint8_t reg, uint8_t value){
    Wire.beginTransmission(Config::MPU6050_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

int16_t ImuManager::readGyroZRaw(){
    Wire.beginTransmission(Config::MPU6050_ADDRESS);
    Wire.write(Config::REG_GYRO_ZOUT_H);
    Wire.endTransmission(false);

    Wire.requestFrom(Config::MPU6050_ADDRESS, static_cast<uint8_t>(2));

    if (Wire.available() < 2) {
        return 0;
    }

    uint8_t highByte = Wire.read();
    uint8_t lowByte = Wire.read();

    return static_cast<int16_t>((highByte << 8) | lowByte);
}

float ImuManager::normalizeHeading(float headingDegValue) const{
    while (headingDegValue > Config::HEADING_MAX_DEG) {
        headingDegValue -= Config::FULL_ROTATION_DEG;
    }

    while (headingDegValue < Config::HEADING_MIN_DEG) {
        headingDegValue += Config::FULL_ROTATION_DEG;
    }

    return headingDegValue;
}