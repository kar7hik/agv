#include <Arduino.h>

#include "Stepper.h"
#include "Imu.h"
#include "Motion.h"
#include "SerialProtocol.h"

Stepper stepper;
Imu imu;
Motion motion(stepper, imu);
SerialProtocol serial(stepper, imu, motion);

void setup() {
    stepper.begin();
    stepper.disable();

    imu.begin();

    motion.begin();

    serial.begin();
}

void loop() {
    serial.update();

    imu.update();

    motion.update();

    serial.sendStatusIfDue();
}