#include "Imu.h"
#include "Config.h"

Imu::Imu()
    : headingDeg(0.0f),
      gyroRateDegPerSec(0.0f),
      gyroBiasDegPerSec(0.0f),
      lastUpdateMicros(0),
      calibrated(false) {
}

bool Imu::begin() {
    Wire.begin();

    if (!writeRegister(Config::REG_PWR_MGMT_1, 0x00)) {
        return false;
    }

    delay(100);

    if (!writeRegister(Config::REG_GYRO_CONFIG, 0x00)) {
        return false;
    }

    delay(50);

    lastUpdateMicros = micros();

    return true;
}

bool Imu::calibrate() {
    int32_t sum = 0;

    for (uint16_t i = 0; i < Config::IMU_CALIBRATION_SAMPLES; ++i) {
        int16_t raw;

        if (!readGyroRaw(raw)) {
            calibrated = false;
            return false;
        }

        sum += raw;
        delay(Config::IMU_CALIBRATION_DELAY_MS);
    }

    const float averageRaw =
        static_cast<float>(sum) / Config::IMU_CALIBRATION_SAMPLES;

    gyroBiasDegPerSec =
        averageRaw / Config::GYRO_SCALE_250DPS;

    headingDeg = 0.0f;
    gyroRateDegPerSec = 0.0f;
    lastUpdateMicros = micros();

    calibrated = true;

    return true;
}

void Imu::update() {
    if (!calibrated) {
        return;
    }

    const uint32_t now = micros();

    const float dt =
        static_cast<float>(now - lastUpdateMicros) / 1000000.0f;

    lastUpdateMicros = now;

    if (dt <= 0.0f || dt > 0.1f) {
        return;
    }

    int16_t raw;

    if (!readGyroRaw(raw)) {
        return;
    }

    gyroRateDegPerSec =
        (static_cast<float>(raw) / Config::GYRO_SCALE_250DPS) - gyroBiasDegPerSec;

    headingDeg += gyroRateDegPerSec * dt;

    headingDeg = normalizeAngle(headingDeg);
}

void Imu::resetHeading() {
    headingDeg = 0.0f;
    lastUpdateMicros = micros();
}

float Imu::getHeading() const {
    return headingDeg;
}

float Imu::getGyroRate() const {
    return gyroRateDegPerSec;
}

bool Imu::isCalibrated() const {
    return calibrated;
}

bool Imu::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(Config::MPU6050_ADDRESS);
    Wire.write(reg);
    Wire.write(value);

    return Wire.endTransmission() == 0;
}

bool Imu::readGyroRaw(int16_t& raw) {
    Wire.beginTransmission(Config::MPU6050_ADDRESS);
    Wire.write(Config::REG_GYRO_ZOUT_H);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom(Config::MPU6050_ADDRESS,
                         static_cast<uint8_t>(2))
        != 2) {
        return false;
    }

    raw = static_cast<int16_t>(
        (Wire.read() << 8) | Wire.read());

    return true;
}

float Imu::normalizeAngle(float angleDeg) {
    while (angleDeg > Config::HEADING_MAX_DEG) {
        angleDeg -= Config::FULL_CIRCLE_DEG;
    }

    while (angleDeg <= Config::HEADING_MIN_DEG) {
        angleDeg += Config::FULL_CIRCLE_DEG;
    }

    return angleDeg;
}