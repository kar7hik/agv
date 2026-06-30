#include <Arduino.h>
#include "ImuManager.h"

ImuManager imu;

void setup(){
    Serial.begin(115200);

    if (!imu.begin()){
        Serial.println("IMU init failed.");

        while (true);
    }

    Serial.println("Send: imu_calibrate");
}

void loop(){
    if (Serial.available()){
        String cmd =
            Serial.readStringUntil('\n');

        cmd.trim();

        if (cmd == "imu_calibrate"){
            imu.calibrate();

            Serial.println(
                "IMU_CALIBRATED"
            );
        }
    }

    imu.update();

    if (imu.isCalibrated()){
        Serial.printf("imu,%.2f\n",imu.getHeadingDeg());
    }

    delay(10);
}