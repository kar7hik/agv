#pragma once

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "Config.h"
#include "MotorManager.h"
#include "ImuManager.h"
#include "MotionController.h"

class SerialProtocol {
public:
    SerialProtocol(MotorManager& motorManager, ImuManager& imuManager, MotionController& motionController);

    void begin();
    void update();

    void sendStatusIfDue();

private:
    void readSerialCommand();
    void processCommand(char* line);

    void handleVelocityCommand();
    void printStatus();
    void printHelp();

private:
    MotorManager& motor;
    ImuManager& imu;
    MotionController& motion;

    char commandBuffer[96];
    uint8_t commandIndex;

    uint32_t lastCommandMillis;
    uint32_t lastStatusMillis;

    bool commandActive;
};