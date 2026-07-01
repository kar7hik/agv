#pragma once

#include <Arduino.h>

#include "Config.h"
#include "Stepper.h"
#include "Imu.h"
#include "Motion.h"

class SerialProtocol {
public:
    SerialProtocol(
        Stepper& stepper,
        Imu& imu,
        Motion& motion);

    void begin();

    void update();

    void sendStatusIfDue();

private:
    void readCommand();

    void processCommand(char* line);

    void handleVelocityCommand();

    void printStatus();

    void printHelp();

private:
    Stepper& stepper;
    Imu& imu;
    Motion& motion;

    char commandBuffer[128];

    uint8_t commandIndex;

    uint32_t lastCommandTime;
    uint32_t lastStatusTime;

    bool commandActive;
};