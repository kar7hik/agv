#include <Arduino.h>

#include "Config.h"
#include "MotorManager.h"
#include "ImuManager.h"
#include "MotionController.h"
#include "SerialProtocol.h"

MotorManager motorManager;
ImuManager imuManager;
MotionController motionController(motorManager, imuManager);
SerialProtocol serialProtocol(motorManager, imuManager, motionController);
void setup(){
    motorManager.begin();
    motorManager.disableDrivers();

    imuManager.begin();

    motionController.begin();

    serialProtocol.begin();
}

void loop(){
    serialProtocol.update();

    imuManager.update();

    motionController.update();

    serialProtocol.sendStatusIfDue();
}