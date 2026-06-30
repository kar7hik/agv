#pragma once
#include "Arduino.h"
#include <Wire.h>

class ImuManager{
    public:
        /* Initialize MPU6050 and internal state 
        Returns true if initialization succeeds.*/

        bool begin();

        /* Perform Gyro caliberation while stationary.*/

        bool calibrate();
        
        /* Update IMU measurements and orientation estimate*/

        void update();

        /* Relative heading used for navigation.
        Range: -180 to +180 degrees*/

        float getHeadingDeg() const;

        /* Raw gyro Z rate. */

        float getGyroZDegPerSec() const;

        /* Whether IMU initialization succedded.*/

        bool isInitialized() const;

        /* Whether calibration is completed.*/

        bool isCalibrated() const;

    private:
        /* Sensor state */

        bool _initialized = false;
        bool _calibrated = false;

        float _gyroBiasZDegPerSec = 0.0f;

        float _gyroZDegPerSec = 0.0f;
        
        float _relativeHeadingDeg = 0.0f;

        uint32_t _lastUpdateMicros = 0;

        bool readRawGyroZ(float &gyroZDegPerSec);

        float wrapAngleDeg(float angleDeg);
};