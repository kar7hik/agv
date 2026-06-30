#pragma once

#include <Arduino.h>

namespace Config {

// ---------------- Serial ----------------
constexpr uint32_t SERIAL_BAUD = 115200;

// ---------------- Motor pins ----------------
constexpr uint8_t LEFT_STEP_PIN  = 16;
constexpr uint8_t LEFT_DIR_PIN   = 26;
constexpr uint8_t LEFT_EN_PIN    = 25;

constexpr uint8_t RIGHT_STEP_PIN = 4;
constexpr uint8_t RIGHT_DIR_PIN  = 13;
constexpr uint8_t RIGHT_EN_PIN   = 14;

// Set this according to your stepper driver enable polarity.
// Your existing test code enabled the drivers with LOW.
constexpr uint8_t DRIVER_ENABLE_LEVEL  = LOW;
constexpr uint8_t DRIVER_DISABLE_LEVEL = HIGH;

// Change these only if motor wiring or mechanical direction is reversed.
constexpr bool LEFT_DIR_INVERTED  = false;
constexpr bool RIGHT_DIR_INVERTED = false;

// ---------------- ESP32 hardware timers ----------------
constexpr uint8_t LEFT_TIMER_ID  = 0;
constexpr uint8_t RIGHT_TIMER_ID = 1;
constexpr uint16_t TIMER_PRESCALER = 80;   // 80 MHz / 80 = 1 MHz, so 1 tick = 1 us

// ---------------- MPU6050 ----------------
constexpr uint8_t MPU6050_ADDR = 0x68;
constexpr uint8_t REG_PWR_MGMT_1  = 0x6B;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_GYRO_ZOUT_H = 0x47;

constexpr float GYRO_SCALE_250DPS = 131.0f;   // raw LSB per deg/s at +/-250 deg/s
constexpr uint16_t IMU_CALIBRATION_SAMPLES = 1000;
constexpr uint16_t IMU_CALIBRATION_DELAY_MS = 2;
constexpr float IMU_HEADING_CORRECTION_ALPHA = 0.03f; // slow visual correction, not hard reset

// ---------------- Motion limits ----------------
constexpr float MAX_VELOCITY_MPS = 0.60f;
constexpr float MAX_MOTOR_FREQ_HZ = 6000.0f;
constexpr float MIN_EFFECTIVE_FREQ_HZ = 1.0f;

// Must match your real drivetrain.
constexpr float WHEEL_DIAMETER_M = 0.065f;
constexpr uint16_t MOTOR_FULL_STEPS_PER_REV = 200;
constexpr uint16_t MICROSTEPS = 16;
constexpr float GEAR_RATIO = 1.0f;

constexpr float STEPS_PER_WHEEL_REV = MOTOR_FULL_STEPS_PER_REV * MICROSTEPS * GEAR_RATIO;
constexpr float WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * 3.14159265358979323846f;
constexpr float STEPS_PER_METER = STEPS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_M;

// One sign constant for steering. If correction turns the wrong way, change only this.
constexpr float STEERING_DIRECTION = 1.0f;

// Controller gains. Units convert navigation errors into step-frequency difference.
constexpr float HEADING_KP_HZ_PER_DEG = 18.0f;
constexpr float LATERAL_KP_HZ_PER_M = 1800.0f;
constexpr float MAX_STEERING_HZ = 1200.0f;

// Controller update rate and safety timeout.
constexpr uint32_t CONTROL_PERIOD_US = 2500;      // 400 Hz
constexpr uint32_t COMMAND_TIMEOUT_MS = 300;
constexpr uint32_t STATUS_PERIOD_MS = 100;

// ---------------- Protocol ----------------
constexpr uint8_t PACKET_SOF_1 = 0xAA;
constexpr uint8_t PACKET_SOF_2 = 0x55;
constexpr uint8_t MAX_PAYLOAD_SIZE = 32;

} // namespace Config
