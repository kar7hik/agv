#include <Arduino.h>
#include "Config.h"
#include "MotorManager.h"
#include "ImuManager.h"
#include "MotionController.h"
#include "SerialProtocol.h"

MotorManager motorManager;
ImuManager imuManager;
MotionController motionController;
SerialProtocol serialProtocol;

void setup() {
    // SerialProtocol.begin() starts Serial at Config::SERIAL_BAUD.
    Serial.begin(Config::SERIAL_BAUD);
    delay(100);

    const bool motorsOk = motorManager.begin();
    const bool imuOk = imuManager.begin();
    const bool controllerOk = motionController.begin(&motorManager, &imuManager);
    const bool serialOk = serialProtocol.begin(Serial, &motionController, &imuManager, &motorManager);

    if (!motorsOk || !imuOk || !controllerOk || !serialOk) {
        motionController.setFault();
        serialProtocol.sendFault(10);
    }

    // Safe default. Raspberry Pi must explicitly enable motors.
    motorManager.disableDrivers();
}

void loop() {
    serialProtocol.update();
    imuManager.update();

    if (imuManager.hasFault()) {
        motionController.setFault();
    }

    motionController.update();
    serialProtocol.sendStatusIfDue();
}
