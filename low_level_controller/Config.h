#pragma once

#include <Arduino.h>
#include <math.h>

namespace Config {
    // Serial //
    constexpr uint32_t SERIAL_BAUD_RATE = 115200;

    // Motor Pins //
    constexpr uint8_t LEFT_STEP_PIN = 16;
    constexpr uint8_t LEFT_DIR_PIN = 26;
    constexpr uint8_t LEFT_EN_PIN = 25;

    constexpr uint8_t RIGHT_STEP_PIN = 4;
    constexpr uint8_t RIGHT_DIR_PIN = 13;
    constexpr uint8_t RIGHT_EN_PIN = 14;
    
    // Wheel config //
    constexpr float WHEEL_DIAMETER_M = 0.117f;
    constexpr uint16_t PULSES_PER_REV = 20000;
    constexpr float WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * M_PI;

    constexpr float PULSES_PER_METER = (PULSES_PER_REV / WHEEL_CIRCUMFERENCE_M);

    // Motor Limits //
    constexpr float MAX_VELOCITY_MPS = 0.10f; // robot max linear speed in m/s
    constexpr float MAX_MOTOR_FREQ_HZ = 10000.0f; // the vel converted to motor freq.
    constexpr float MIN_EFFECTIVE_FREQ_HZ = 1.0f; 


    // Driver enable / disable //
    constexpr uint8_t DRIVER_ENABLE_LEVEL = LOW;
    constexpr uint8_t DRIVER_DISABLE_LEVEL = HIGH;

    // Esp32 hardware timers //
    constexpr uint8_t LEFT_TIMER_ID = 0;
    constexpr uint8_t RIGHT_TIMER_ID = 1;
    constexpr uint16_t TIMER_PRESCALER = 80; 

    // Initial timer alarm value //

    constexpr uint32_t INITIAL_TIMER_ALARM_US = 1000;

    // MPU6050 //
    constexpr uint8_t MPU6050_ADDRESS = 0x68;
    constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
    constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
    constexpr uint8_t REG_GYRO_ZOUT_H = 0x47;

    constexpr float GYRO_SCALE_250DPS = 131.0f;   // raw LSB per deg/s at +/-250 deg/s
    constexpr uint16_t IMU_CALIBRATION_SAMPLES = 1000;
    constexpr uint16_t IMU_CALIBRATION_DELAY_MS = 2;

    // Heading //
    constexpr float HEADING_MAX_DEG = 180.0f;
    constexpr float HEADING_MIN_DEG = -180.0f;
    constexpr float FULL_ROTATION_DEG = 360.0f;

    // Controller update rate and safety timeout //
    constexpr uint32_t CONTROL_PERIOD_US = 2500;    // 400 Hz 
    constexpr uint32_t COMMAND_TIMEOUT_MS = 300;    // when no command is received for 300 ms, stop motors
    constexpr uint32_t STATUS_PERIOD_MS = 100;      // send status every 100 ms
    // Protocol //
    constexpr uint8_t PACKET_SOF_1 = 0xAA;
    constexpr uint8_t PACKET_SOF_2 = 0x55;

    // =====================================================
    // Controller Gains
    // =====================================================

    // These values are for MotionController.
    // Do not use them inside MotorManager.

    // HEADING_KP_HZ_PER_DEG:
    //   Steering correction from heading error.
    //
    //   Example:
    //      heading error = 5 deg
    //      steering = 5 * 18 = 90 Hz
    //
    //   Tune:
    //      If robot corrects too slowly, increase this.
    //      If robot oscillates left-right, decrease this.
    constexpr float HEADING_KP_HZ_PER_DEG = 18.0f;

    // LATERAL_KP_HZ_PER_M:
    //   Steering correction from lateral path error.
    //
    //   Example:
    //      lateral error = 0.02 m
    //      steering = 0.02 * 1800 = 36 Hz
    //
    //   Tune:
    //      If robot does not return to path, increase this.
    //      If robot snakes/oscillates near path, decrease this.
    constexpr float LATERAL_KP_HZ_PER_M = 1800.0f;

    // MAX_STEERING_HZ:
    //   Maximum allowed steering correction.
    //   Prevents one bad AprilTag reading from causing a huge turn.
    //
    //   Tune:
    //      Start around 20 percent of MAX_MOTOR_FREQ_HZ.
    //      If correction is too weak, increase slowly.
    //      If turns become jerky, decrease it.
    constexpr float MAX_STEERING_HZ = 1200.0f;

    // STEERING_DIRECTION:
    //   Change only this if correction direction is reversed.
    //   Use 1.0 first.
    //   If robot corrects opposite, change to -1.0.
    //   Do not invert signs in many places.
    constexpr float STEERING_DIRECTION = 1.0f;

    constexpr uint32_t VISUAL_CORRECTION_TIMEOUT_MS = 200;
}

