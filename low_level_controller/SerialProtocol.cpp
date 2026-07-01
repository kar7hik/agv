#include "SerialProtocol.h"

SerialProtocol::SerialProtocol(MotorManager& motorManager, ImuManager& imuManager, MotionController& motionController)
    : motor(motorManager), imu(imuManager), motion(motionController),
      commandIndex(0), lastCommandMillis(0), lastStatusMillis(0),
      commandActive(false){
        commandBuffer[0] = '\0';
}

void SerialProtocol::begin(){
    Serial.begin(Config::SERIAL_BAUD_RATE);
    delay(1000);

    lastCommandMillis = millis();
    lastStatusMillis = millis();

    Serial.println();
    Serial.println("AGV ESP32 SerialProtocol Ready");
    printHelp();
}

void SerialProtocol::update(){
    readSerialCommand();

    if (commandActive) {
        if (millis() - lastCommandMillis > Config::COMMAND_TIMEOUT_MS) {
            motion.stop();
            commandActive = false;

            Serial.println("FAULT command timeout");
        }
    }
}

void SerialProtocol::sendStatusIfDue(){
    if (millis() - lastStatusMillis >= Config::STATUS_PERIOD_MS) {
        lastStatusMillis = millis();
        printStatus();
    }
}

void SerialProtocol::readSerialCommand(){
    while (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());

        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            commandBuffer[commandIndex] = '\0';

            if (commandIndex > 0) {
                processCommand(commandBuffer);
            }

            commandIndex = 0;
            return;
        }

        if (commandIndex < sizeof(commandBuffer) - 1) {
            commandBuffer[commandIndex] = c;
            commandIndex++;
        } else {
            commandIndex = 0;
            commandBuffer[0] = '\0';
            Serial.println("ERR command too long");
        }
    }
}

void SerialProtocol::processCommand(char* line){
    char* cmd = strtok(line, " ");

    if (cmd == nullptr) {
        return;
    }

    if (strcmp(cmd, "HELP") == 0 || strcmp(cmd, "help") == 0) {
        printHelp();
        return;
    }

    if (strcmp(cmd, "PING") == 0 || strcmp(cmd, "ping") == 0) {
        Serial.println("ACK PING");
        return;
    }

    if (strcmp(cmd, "EN") == 0 || strcmp(cmd, "en") == 0) {
        motor.enableDrivers();
        Serial.println("ACK EN");
        return;
    }

    if (strcmp(cmd, "DIS") == 0 || strcmp(cmd, "dis") == 0) {
        motion.stop();
        motor.disableDrivers();
        commandActive = false;

        Serial.println("ACK DIS");
        return;
    }

    if (strcmp(cmd, "STOP") == 0 || strcmp(cmd, "stop") == 0) {
        motion.stop();
        commandActive = false;

        Serial.println("ACK STOP");
        return;
    }

    if (strcmp(cmd, "CAL") == 0 || strcmp(cmd, "cal") == 0) {
        motion.stop();
        commandActive = false;

        Serial.println("ACK CAL START");
        Serial.println("Keep robot stationary");

        imu.calibrate();

        Serial.println("ACK CAL DONE");
        return;
    }

    if (strcmp(cmd, "ZERO") == 0 || strcmp(cmd, "zero") == 0) {
        imu.resetHeading();

        Serial.println("ACK ZERO");
        return;
    }

    if (strcmp(cmd, "STATUS") == 0 || strcmp(cmd, "status") == 0) {
        printStatus();
        return;
    }

    if (strcmp(cmd, "VEL") == 0 || strcmp(cmd, "vel") == 0) {
        handleVelocityCommand();
        return;
    }

    Serial.print("ERR unknown command: ");
    Serial.println(cmd);
}

void SerialProtocol::handleVelocityCommand(){
    char* velStr = strtok(nullptr, " ");
    char* headingStr = strtok(nullptr, " ");
    char* lateralStr = strtok(nullptr, " ");

    if (velStr == nullptr || headingStr == nullptr || lateralStr == nullptr) {
        Serial.println("ERR usage: VEL velocity headingError lateralError");
        Serial.println("Example: VEL 0.05 -3 0.01");
        return;
    }

    float velocityMps = atof(velStr);
    float headingErrorDeg = atof(headingStr);
    float lateralErrorM = atof(lateralStr);

    motor.enableDrivers();

    motion.setNavigationCommand(
        velocityMps,
        headingErrorDeg,
        lateralErrorM
    );

    lastCommandMillis = millis();

    if (fabsf(velocityMps) > 0.0f) {
        commandActive = true;
    } else {
        commandActive = false;
    }

    Serial.print("ACK VEL ");
    Serial.print(velocityMps, 3);
    Serial.print(" ");
    Serial.print(headingErrorDeg, 3);
    Serial.print(" ");
    Serial.println(lateralErrorM, 4);
}

void SerialProtocol::printStatus(){
    Serial.print("STATUS ");

    Serial.print("mode=");

    MotionController::Mode mode = motion.getMode();

    if (mode == MotionController::Mode::Stopped) {
        Serial.print("STOPPED");
    } else if (mode == MotionController::Mode::VisualCorrection) {
        Serial.print("VISUAL");
    } else if (mode == MotionController::Mode::ImuHeadingHold) {
        Serial.print("IMU_HOLD");
    }

    Serial.print(" en=");
    Serial.print(motor.areDriversEnabled() ? 1 : 0);

    Serial.print(" imuCal=");
    Serial.print(imu.isCalibrated() ? 1 : 0);

    Serial.print(" imu=");
    Serial.print(imu.getHeadingDeg(), 2);

    Serial.print(" gyroZ=");
    Serial.print(imu.getGyroZDegPerSec(), 2);

    Serial.print(" target=");
    Serial.print(motion.getTargetHeadingDeg(), 2);

    Serial.print(" imuErr=");
    Serial.print(motion.getImuHeadingErrorDeg(), 2);

    Serial.print(" vel=");
    Serial.print(motion.getCommandVelocityMps(), 3);

    Serial.print(" visualH=");
    Serial.print(motion.getVisualHeadingErrorDeg(), 2);

    Serial.print(" visualL=");
    Serial.print(motion.getVisualLateralErrorM(), 4);

    Serial.print(" L=");
    Serial.print(motion.getLeftFrequencyHz(), 1);

    Serial.print(" R=");
    Serial.println(motion.getRightFrequencyHz(), 1);
}

void SerialProtocol::printHelp(){
    Serial.println();
    Serial.println("Commands:");
    Serial.println("  HELP");
    Serial.println("  PING");
    Serial.println("  EN");
    Serial.println("  DIS");
    Serial.println("  STOP");
    Serial.println("  CAL");
    Serial.println("  ZERO");
    Serial.println("  STATUS");
    Serial.println("  VEL velocity headingError lateralError");
    Serial.println();
    Serial.println("Examples:");
    Serial.println("  PING");
    Serial.println("  EN");
    Serial.println("  CAL");
    Serial.println("  VEL 0.05 0 0");
    Serial.println("  VEL 0.05 -3 0.01");
    Serial.println("  VEL 0 0 0");
    Serial.println("  STOP");
    Serial.println();
}