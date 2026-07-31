#include <Arduino.h>
#include <string.h>
#include <Wire.h>
#include <stdlib.h>
#include <math.h>

// Serial
constexpr size_t COMMAND_BUFFER_SIZE = 128;
char commandBuffer[COMMAND_BUFFER_SIZE];
size_t commandIndex = 0;

// Motor Pins
constexpr uint8_t LEFT_STEP_PIN = 16;
constexpr uint8_t LEFT_DIR_PIN = 26;
constexpr uint8_t LEFT_EN_PIN = 25;

constexpr uint8_t RIGHT_STEP_PIN = 4;
constexpr uint8_t RIGHT_DIR_PIN = 13;
constexpr uint8_t RIGHT_EN_PIN = 14;

constexpr uint8_t MOTOR_ENABLE_LEVEL = LOW;
constexpr uint8_t MOTOR_DISABLE_LEVEL = HIGH;

constexpr uint8_t LEFT_FORWARD_LEVEL = HIGH;
constexpr uint8_t RIGHT_FORWARD_LEVEL = HIGH;


// Robot Parameters:
constexpr float PI_F = 3.14159265358979323846f;

constexpr float WHEEL_DIAMETER_M = 0.117f;
constexpr uint32_t PULSES_PER_REV = 20000;

constexpr float WHEEL_CIRCUMFERENCE_M = PI_F * WHEEL_DIAMETER_M;
constexpr float PULSES_PER_METER = PULSES_PER_REV / WHEEL_CIRCUMFERENCE_M;
constexpr float TRACK_WIDTH_M = 0.355f;

// Limits:
constexpr float MAX_WHEEL_VELOCITY_MPS = 0.10f;
constexpr float MAX_STEP_RATE_PPS = 6000.0f;
constexpr float MIN_STEP_RATE_PPS = 1.0f;

constexpr uint32_t INITIAL_TIMER_PERIOD_US = 1000;

// Motor state:
bool motorsEnabled = false;

hw_timer_t *leftTimer = nullptr;
hw_timer_t *rightTimer = nullptr;

volatile bool leftStepState = false;
volatile bool rightStepState = false;

volatile uint32_t leftPulseCount = 0;
volatile uint32_t rightPulseCount = 0;

float leftVelocityMps = 0.0f;
float rightVelocityMps = 0.0f;

float leftStepRatePps = 0.0f;
float rightStepRatePps = 0.0f;

// Motion Control Constants:
constexpr float MAX_LINEAR_VELOCITY_MPS = 0.10f;
constexpr float MAX_ANGULAR_VELOCITY_RAD_S = 0.40f;

constexpr float LINEAR_ACCEL_MPS2 = 0.06f;
constexpr float LINEAR_DECEL_MPS2 = 0.10f;

constexpr float ANGULAR_ACCEL_RAD_S2 = 0.80f;
constexpr uint32_t MOTION_CONTROL_PERIOD_US = 1000;

float targetLinearVelocityMps = 0.0f;
float targetAngularVelocityRadS = 0.0f;

float currentLinearVelocityMps = 0.0f;
float currentAngularVelocityRadS = 0.0f;

uint32_t lastMotionControlTimeUs = 0;
bool motionControlEnabled = false;

// IMU Constants:
constexpr uint8_t MPU6050_ADDRESS = 0x68;
constexpr uint8_t MPU6050_PWR_MGMT_1 = 0x6B;
constexpr uint8_t MPU6050_GYRO_CONFIG = 0x1B;
constexpr uint8_t MPU6050_GYRO_ZOUT_H = 0x47;
constexpr uint8_t MPU6050_WHO_AM_I = 0x75;

constexpr float GYRO_SCALE_LSB_PER_DPS = 131.0f;
constexpr uint32_t IMU_UPDATE_PERIOD_US = 5000;

// IMU State Variables:
bool imuReady = false;
bool imuCalibrated = false;

float gyroZCorrectionDps = 0.0f;
float imuHeadingDeg = 0.0f;
float headingCorrectionDeg = 0.0f;
uint32_t lastImuUpdateUs = 0;

// Drive Heading Control:
constexpr float DRIVE_HEADING_KP = 1.5f;

constexpr float LATERAL_CORRECTION_DISTANCE_M = 0.5625f;

// Prevent an excessive steering angle from a large/noisy tag error.
constexpr float MAX_DRIVE_HEADING_OFFSET_DEG = 15.0f;

bool driveActive = false;

float driveVelocityMps = 0.0f;
float drivePathHeadingDeg = 0.0f;

float driveLateralErrorM = 0.0f;
float driveInitialHeadingErrorDeg = 0.0f;

// Current heading offset produced by the cubic trajectory.
float driveHeadingOffsetDeg = 0.0f;

uint32_t driveStartLeftPulseCount = 0;
uint32_t driveStartRightPulseCount = 0;


// Utility Functions:
float clampFloat(float value, float low, float high) {
    if (value < low) {
        return low;
    }

    if (value > high) {
        return high;
    }

    return value;
}

uint8_t directionLevel(float value, uint8_t forwardLevel) {
    if (value > 0.0f) {
        return forwardLevel;
    } else {
        return !forwardLevel;
    }
}

float rampToward(float current, float target, float maxChange) {
    if (current < target) {
        current += maxChange;
        if (current > target) {
            current = target;
        }
    } else if (current > target) {
        current -= maxChange;
        if (current < target) {
            current = target;
        }
    }

    return current;
}

// To keep the angle between -180 and 180 degrees:
float normalizeAngleDeg(float angleDeg) {
    while (angleDeg > 180.0f) {
        angleDeg -= 360.0f;
    }

    while (angleDeg < -180.0f) {
        angleDeg += 360.0f;
    }

    return angleDeg;
}


// IMU
bool writeImuRegister(uint8_t registerAddress, uint8_t value) {
    Wire.beginTransmission(MPU6050_ADDRESS);

    Wire.write(registerAddress);
    Wire.write(value);

    return Wire.endTransmission() == 0;
}

bool readImuRegister(uint8_t registerAddress, uint8_t &value) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(registerAddress);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom(MPU6050_ADDRESS, static_cast<uint8_t>(1)) != 1) {
        return false;
    }

    value = Wire.read();
    return true;
}

bool setupImu() {
    Wire.begin();
    delay(100);

    uint8_t deviceId = 0;
    if (!readImuRegister(MPU6050_WHO_AM_I, deviceId)) {
        return false;
    }

    if (deviceId != 0x68) {
        return false;
    }

    // Wake the MPU6050
    if (!writeImuRegister(MPU6050_PWR_MGMT_1, 0x01)) {
        return false;
    }

    delay(100);

    // Gyroscope range: +/- 250 degrees per second
    if (!writeImuRegister(MPU6050_GYRO_CONFIG, 0x00)) {
        return false;
    }

    lastImuUpdateUs = micros();
    imuReady = true;

    return true;
}

bool readGyroZRaw(int16_t &gyroZRaw) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPU6050_GYRO_ZOUT_H);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom(MPU6050_ADDRESS, static_cast<uint8_t>(2)) != 2) {
        return false;
    }

    const uint8_t highByte = Wire.read();
    const uint8_t lowByte = Wire.read();

    gyroZRaw = static_cast<int16_t>(
        (static_cast<uint16_t>(highByte) << 8) | lowByte);

    return true;
}


bool calibrateImu() {
    if (!imuReady) {
        return false;
    }

    stopImmediate();

    constexpr uint16_t CALIBRATION_SAMPLES = 2000;

    int64_t gyroSum = 0;
    uint16_t validSamples = 0;

    for (uint16_t i = 0; i < CALIBRATION_SAMPLES; i++) {
        int16_t gyroZRaw = 0;

        if (readGyroZRaw(gyroZRaw)) {
            gyroSum += gyroZRaw;
            validSamples++;
        }

        delay(2);
    }

    if (validSamples == 0) {
        return false;
    }

    const float averageRaw = static_cast<float>(gyroSum) / static_cast<float>(validSamples);

    gyroZCorrectionDps = averageRaw / GYRO_SCALE_LSB_PER_DPS;

    imuHeadingDeg = 0.0f;
    headingCorrectionDeg = 0.0f;

    lastImuUpdateUs = micros();
    imuCalibrated = true;

    return true;
}

void updateImu() {
    if (!imuReady || !imuCalibrated) {
        return;
    }

    const uint32_t nowUs = micros();
    const uint32_t elapsedUs = nowUs - lastImuUpdateUs;

    if (elapsedUs < IMU_UPDATE_PERIOD_US) {
        return;
    }

    int16_t gyroZRaw = 0;
    if (!readGyroZRaw(gyroZRaw)) {
        return;
    }

    // Update only after receiving a valid sample.
    lastImuUpdateUs = nowUs;

    const float dt = static_cast<float>(elapsedUs) / 1000000.0f;
    const float measuredGyroZDps = static_cast<float>(gyroZRaw) / GYRO_SCALE_LSB_PER_DPS;
    const float correctedGyroZDps = measuredGyroZDps - gyroZCorrectionDps;

    imuHeadingDeg += correctedGyroZDps * dt;
    imuHeadingDeg = normalizeAngleDeg(imuHeadingDeg);
}


void zeroImuHeading() {
    imuHeadingDeg = 0.0f;
    headingCorrectionDeg = 0.0f;
    lastImuUpdateUs = micros();
}


void syncHeadingFromTag(float trueHeadingDeg) {
    headingCorrectionDeg = normalizeAngleDeg(trueHeadingDeg - imuHeadingDeg);
}

float robotHeadingDeg() {
    return normalizeAngleDeg(imuHeadingDeg + headingCorrectionDeg);
}

// Timer Interrupts:
void IRAM_ATTR onLeftTimer() {
    leftStepState = !leftStepState;
    digitalWrite(LEFT_STEP_PIN, leftStepState);

    // Counting only the rising edge:
    if (leftStepState) {
        leftPulseCount++;
    }
}

void IRAM_ATTR onRightTimer() {
    rightStepState = !rightStepState;
    digitalWrite(RIGHT_STEP_PIN, rightStepState);

    // Counting only the rising edge:
    if (rightStepState) {
        rightPulseCount++;
    }
}

// Timer Setup:
void setupTimers() {
    leftTimer = timerBegin(1000000);
    rightTimer = timerBegin(1000000);

    timerAttachInterrupt(leftTimer, &onLeftTimer);
    timerAttachInterrupt(rightTimer, &onRightTimer);

    timerAlarm(leftTimer, INITIAL_TIMER_PERIOD_US, true, 0);
    timerAlarm(rightTimer, INITIAL_TIMER_PERIOD_US, true, 0);

    timerStop(leftTimer);
    timerStop(rightTimer);
}

void setupMotors() {
    pinMode(LEFT_STEP_PIN, OUTPUT);
    pinMode(LEFT_DIR_PIN, OUTPUT);
    pinMode(LEFT_EN_PIN, OUTPUT);

    pinMode(RIGHT_STEP_PIN, OUTPUT);
    pinMode(RIGHT_DIR_PIN, OUTPUT);
    pinMode(RIGHT_EN_PIN, OUTPUT);

    // To avoid floating or unexpected step signal, set step pins to low:
    digitalWrite(LEFT_STEP_PIN, LOW);
    digitalWrite(RIGHT_STEP_PIN, LOW);

    // Disable motors by default:
    digitalWrite(LEFT_EN_PIN, MOTOR_DISABLE_LEVEL);
    digitalWrite(RIGHT_EN_PIN, MOTOR_DISABLE_LEVEL);

    motorsEnabled = false;
}

void motorOn() {
    digitalWrite(LEFT_EN_PIN, MOTOR_ENABLE_LEVEL);
    digitalWrite(RIGHT_EN_PIN, MOTOR_ENABLE_LEVEL);
    motorsEnabled = true;
}

void stopLeftMotor() {
    timerStop(leftTimer);

    digitalWrite(LEFT_STEP_PIN, LOW);
    leftStepState = false;

    leftVelocityMps = 0.0f;
    leftStepRatePps = 0.0f;
}

void stopRightMotor() {
    timerStop(rightTimer);

    digitalWrite(RIGHT_STEP_PIN, LOW);
    rightStepState = false;

    rightVelocityMps = 0.0f;
    rightStepRatePps = 0.0f;
}


void stopMotion() {
    stopLeftMotor();
    stopRightMotor();
}

void stopImmediate() {
    driveActive = false;

    driveVelocityMps = 0.0f;
    drivePathHeadingDeg = 0.0f;

    driveLateralErrorM = 0.0f;
    driveInitialHeadingErrorDeg = 0.0f;
    driveHeadingOffsetDeg = 0.0f;

    targetLinearVelocityMps = 0.0f;
    targetAngularVelocityRadS = 0.0f;

    currentLinearVelocityMps = 0.0f;
    currentAngularVelocityRadS = 0.0f;

    motionControlEnabled = false;

    stopMotion();
}

// Stop the step pulses and then disable the motors:
void motorOff() {
    stopImmediate();

    digitalWrite(LEFT_EN_PIN, MOTOR_DISABLE_LEVEL);
    digitalWrite(RIGHT_EN_PIN, MOTOR_DISABLE_LEVEL);
    motorsEnabled = false;
}


// Velocity and step rate conversion:
float velocityToStepRate(float velocityMps) {
    return velocityMps * PULSES_PER_METER;
}

uint32_t stepRateToTimerPeriodUs(float stepRatePps) {
    return static_cast<uint32_t>(1000000.0f / (2.0f * stepRatePps));
}

// Apply step rates:
void applyLeftStepRate(float stepRatePps) {
    if (fabsf(stepRatePps) < MIN_STEP_RATE_PPS) {
        stopLeftMotor();
        return;
    }

    digitalWrite(LEFT_DIR_PIN, directionLevel(stepRatePps, LEFT_FORWARD_LEVEL));
    const float magnitude = clampFloat(fabsf(stepRatePps),
                                       MIN_STEP_RATE_PPS,
                                       MAX_STEP_RATE_PPS);

    const uint32_t periodUs = stepRateToTimerPeriodUs(magnitude);

    leftStepRatePps = stepRatePps >= 0.0f ? magnitude : -magnitude;
    timerAlarm(leftTimer, periodUs, true, 0);
    timerStart(leftTimer);
}

void applyRightStepRate(float stepRatePps) {
    if (fabsf(stepRatePps) < MIN_STEP_RATE_PPS) {
        stopRightMotor();
        return;
    }

    digitalWrite(RIGHT_DIR_PIN, directionLevel(stepRatePps, RIGHT_FORWARD_LEVEL));
    const float magnitude = clampFloat(fabsf(stepRatePps),
                                       MIN_STEP_RATE_PPS,
                                       MAX_STEP_RATE_PPS);

    const uint32_t periodUs = stepRateToTimerPeriodUs(magnitude);

    rightStepRatePps = stepRatePps >= 0.0f ? magnitude : -magnitude;
    timerAlarm(rightTimer, periodUs, true, 0);
    timerStart(rightTimer);
}

// Set wheel velocities:
void setLeftVelocity(float velocityMps) {
    leftVelocityMps = clampFloat(velocityMps,
                                 -MAX_WHEEL_VELOCITY_MPS,
                                 MAX_WHEEL_VELOCITY_MPS);

    const float stepRatePps = velocityToStepRate(leftVelocityMps);
    applyLeftStepRate(stepRatePps);
}


void setRightVelocity(float velocityMps) {
    rightVelocityMps = clampFloat(velocityMps,
                                  -MAX_WHEEL_VELOCITY_MPS,
                                  MAX_WHEEL_VELOCITY_MPS);

    const float stepRatePps = velocityToStepRate(rightVelocityMps);
    applyRightStepRate(stepRatePps);
}

void setWheelVelocities(float leftVelocityMps, float rightVelocityMps) {
    if (!motorsEnabled) {
        stopMotion();
        return;
    }

    setLeftVelocity(leftVelocityMps);
    setRightVelocity(rightVelocityMps);
}



// Differential Drive Kinematics:
void setMotion(float linearVelocityMps, float angularVelocityRadS) {
    if (!motorsEnabled) {
        stopMotion();
        return;
    }

    const float halfTrack = TRACK_WIDTH_M * 0.5f;

    float leftMps = linearVelocityMps - (angularVelocityRadS * halfTrack);
    float rightMps = linearVelocityMps + (angularVelocityRadS * halfTrack);

    const float largestWheelSpeed = fmaxf(fabsf(leftMps), fabsf(rightMps));

    if (largestWheelSpeed > MAX_WHEEL_VELOCITY_MPS) {
        const float scale = MAX_WHEEL_VELOCITY_MPS / largestWheelSpeed;
        leftMps *= scale;
        rightMps *= scale;
    }

    setWheelVelocities(leftMps, rightMps);
}


void setMotionTarget(float linearVelocityMps, float angularVelocityRadS) {
    targetLinearVelocityMps = clampFloat(linearVelocityMps,
                                         -MAX_LINEAR_VELOCITY_MPS,
                                         MAX_LINEAR_VELOCITY_MPS);
    targetAngularVelocityRadS = clampFloat(angularVelocityRadS,
                                           -MAX_ANGULAR_VELOCITY_RAD_S,
                                           MAX_ANGULAR_VELOCITY_RAD_S);

    if (!motionControlEnabled) {
        lastMotionControlTimeUs = micros();
    }

    motionControlEnabled = true;
}


float driveDistanceTravelledM() {
    const uint32_t leftPulseDelta = leftPulseCount - driveStartLeftPulseCount;
    const uint32_t rightPulseDelta = rightPulseCount - driveStartRightPulseCount;
    const float averagePulseDelta = (static_cast<float>(leftPulseDelta) + static_cast<float>(rightPulseDelta)) * 0.5f;

    return averagePulseDelta / PULSES_PER_METER;
}

float calculateDriveHeadingOffsetDeg(float progress) {
    const float s = clampFloat(progress, 0.0f, 1.0f);

    if (s >= 1.0f) {
        return 0.0f;
    }

    const float initialHeadingErrorDeg = driveInitialHeadingErrorDeg * PI_F / 180.0f;

    const float lateralSlope = (driveLateralErrorM / LATERAL_CORRECTION_DISTANCE_M) * (6.0f * s * s - 6.0f * s);
    const float initialHeadingSlope = tanf(initialHeadingErrorDeg) * (3.0f * s * s - 4.0f * s + 1.0f);

    const float totalSlope = lateralSlope + initialHeadingSlope;
    const float headingOffsetDeg = atanf(totalSlope) * 180.0f / PI_F;

    return clampFloat(headingOffsetDeg, -MAX_DRIVE_HEADING_OFFSET_DEG, MAX_DRIVE_HEADING_OFFSET_DEG);
}


float calculateHeadingAngularVelocityRadS(float desiredHeadingDeg,
                                          float gain,
                                          float maximumAngularVelocityRadS) {

    const float headingErrorDeg = normalizeAngleDeg(desiredHeadingDeg - robotHeadingDeg());
    const float headingErrorRad = headingErrorDeg * PI_F / 180.0f;

    float angularVelocityRadS = gain * headingErrorRad;
    const float maximumMagnitude = clampFloat(fabsf(maximumAngularVelocityRadS),
                                              0.0f,
                                              MAX_ANGULAR_VELOCITY_RAD_S);

    angularVelocityRadS = clampFloat(angularVelocityRadS,
                                     -maximumMagnitude,
                                     maximumMagnitude);

    return angularVelocityRadS;
}

void startDrive(float velocityMps, float pathHeadingDeg, float lateralErrorM) {
    driveVelocityMps = clampFloat(velocityMps,
                                  -MAX_LINEAR_VELOCITY_MPS,
                                  MAX_LINEAR_VELOCITY_MPS);

    drivePathHeadingDeg = normalizeAngleDeg(pathHeadingDeg);
    driveLateralErrorM = lateralErrorM;

    /*
    robotHeadingDeg() has already been synchronized
    using the accepted AprilTag heading.

    This is the physical heading error relative to
    the required segment direction.
    */
    driveInitialHeadingErrorDeg = normalizeAngleDeg(robotHeadingDeg() - drivePathHeadingDeg);

    /*
    At progress zero, the cubic desired heading should
    equal the robot's current corrected heading.
    */
    driveHeadingOffsetDeg = driveInitialHeadingErrorDeg;

    // Begin measuring correction distance from this point.
    driveStartLeftPulseCount = leftPulseCount;
    driveStartRightPulseCount = rightPulseCount;
    driveActive = true;
}

void updateDriveControl() {
    if (!driveActive) {
        return;
    }

    if (!motorsEnabled || !imuReady || !imuCalibrated) {
        stopImmediate();
        return;
    }

    const float distanceTravelledM = driveDistanceTravelledM();
    const float progress = clampFloat(distanceTravelledM / LATERAL_CORRECTION_DISTANCE_M,
                                      0.0f,
                                      1.0f);

    driveHeadingOffsetDeg = calculateDriveHeadingOffsetDeg(progress);
    const float desiredHeadingDeg = normalizeAngleDeg(drivePathHeadingDeg + driveHeadingOffsetDeg);
    const float angularVelocityRadS = calculateHeadingAngularVelocityRadS(desiredHeadingDeg,
                                                                          DRIVE_HEADING_KP,
                                                                          MAX_ANGULAR_VELOCITY_RAD_S);

    setMotionTarget(driveVelocityMps, angularVelocityRadS);
}


void updateMotionControl() {
    if (!motionControlEnabled) {
        return;
    }

    if (!motorsEnabled) {
        stopImmediate();
        return;
    }

    const uint32_t nowUs = micros();
    const uint32_t elapsedUs = nowUs - lastMotionControlTimeUs;

    if (elapsedUs < MOTION_CONTROL_PERIOD_US) {
        return;
    }

    lastMotionControlTimeUs = nowUs;

    const float dt = static_cast<float>(elapsedUs) / 1000000.0f;
    const bool changingDirection = (currentLinearVelocityMps * targetLinearVelocityMps) < 0.0f;
    const bool speedingUp = fabsf(targetLinearVelocityMps) > fabsf(currentLinearVelocityMps);

    float linearRate = LINEAR_DECEL_MPS2;
    if (!changingDirection && speedingUp) {
        linearRate = LINEAR_ACCEL_MPS2;
    }

    currentLinearVelocityMps = rampToward(currentLinearVelocityMps,
                                          targetLinearVelocityMps,
                                          linearRate * dt);

    currentAngularVelocityRadS = rampToward(currentAngularVelocityRadS,
                                            targetAngularVelocityRadS,
                                            ANGULAR_ACCEL_RAD_S2 * dt);

    const bool targetStopped = fabsf(targetLinearVelocityMps) < 0.001f && fabsf(targetAngularVelocityRadS) < 0.001f;
    const bool stopped = fabsf(currentLinearVelocityMps) < 0.001f && fabsf(currentAngularVelocityRadS) < 0.001f;
    if (targetStopped && stopped) {
        currentLinearVelocityMps = 0.0f;
        currentAngularVelocityRadS = 0.0f;
        motionControlEnabled = false;
        stopMotion();

        return;
    }

    setMotion(currentLinearVelocityMps, currentAngularVelocityRadS);
}


void printStatus() {
    Serial.print("STATUS MOTOR=");
    Serial.print(motorsEnabled ? "ON" : "OFF");

    Serial.print(" IMU=");
    Serial.print(imuReady ? "READY" : "ERROR");

    Serial.print(" CAL=");
    Serial.print(imuCalibrated ? "YES" : "NO");

    Serial.print(" MODE=");
    Serial.print(driveActive ? "DRIVE" : motionControlEnabled ? "MOTION"
                                                              : "IDLE");

    Serial.print(" RAW=");
    Serial.print(imuHeadingDeg, 2);

    Serial.print(" OFF=");
    Serial.print(headingCorrectionDeg, 2);

    Serial.print(" HDG=");
    Serial.print(robotHeadingDeg(), 2);

    if (driveActive) {
        const float distanceM = driveDistanceTravelledM();
        const float progress = clampFloat(distanceM / LATERAL_CORRECTION_DISTANCE_M,
                                          0.0f,
                                          1.0f);

        Serial.print(" DIST=");
        Serial.print(distanceM, 2);

        Serial.print(" PROGRESS=");
        Serial.print(progress, 2);

        Serial.print(" PATH=");
        Serial.print(drivePathHeadingDeg, 2);

        Serial.print(" LAT=");
        Serial.print(driveLateralErrorM, 4);

        Serial.print(" IERR=");
        Serial.print(driveInitialHeadingErrorDeg, 2);

        Serial.print(" HOFF=");
        Serial.print(driveHeadingOffsetDeg, 2);

        Serial.print(" DHDG=");
        Serial.print(normalizeAngleDeg(drivePathHeadingDeg + driveHeadingOffsetDeg), 2);
    }

    Serial.print(" V=");
    Serial.print(currentLinearVelocityMps, 3);

    Serial.print(" TV=");
    Serial.print(targetLinearVelocityMps, 3);

    Serial.print(" W=");
    Serial.print(currentAngularVelocityRadS, 3);

    Serial.print(" TW=");
    Serial.println(targetAngularVelocityRadS, 3);
}



void processCommand(char *line) {
    char *command = strtok(line, " ");

    if (command == nullptr) {
        return;
    }

    if (!strcmp(command, "PING")) {
        Serial.println("ACK");

        return;
    }

    if (!strcmp(command, "STATUS")) {
        printStatus();
        return;
    }

    if (!strcmp(command, "MOTOR_ON")) {
        motorOn();
        Serial.println("ACK");

        return;
    }

    if (!strcmp(command, "MOTOR_OFF")) {
        motorOff();
        Serial.println("ACK");

        return;
    }

    if (!strcmp(command, "STOP")) {
        stopImmediate();
        Serial.println("ACK");

        return;
    }

    if (!strcmp(command, "CAL_IMU")) {
        Serial.println("CALIBRATING");

        if (!calibrateImu()) {
            Serial.println("ERR IMU_CAL");
            return;
        }

        Serial.println("ACK");

        return;
    }

    if (!strcmp(command, "ZERO_IMU")) {
        if (!imuReady || !imuCalibrated) {
            Serial.println("ERR IMU_NOT_READY");
            return;
        }

        zeroImuHeading();
        Serial.println("ACK");

        return;
    }

    if (!strcmp(command, "SYNC")) {
        char *headingString = strtok(nullptr, " ");

        if (headingString == nullptr) {
            Serial.println("ERR SYNC");
            return;
        }

        if (!imuReady || !imuCalibrated) {
            Serial.println("ERR IMU_NOT_READY");
            return;
        }

        const float trueHeadingDeg = atof(headingString);
        syncHeadingFromTag(trueHeadingDeg);
        Serial.println("ACK");

        return;
    }

    if (!strcmp(command, "DRIVE")) {
        char *velocityString = strtok(nullptr, " ");
        char *headingString = strtok(nullptr, " ");
        char *lateralErrorString = strtok(nullptr, " ");

        if (
            velocityString == nullptr || headingString == nullptr || lateralErrorString == nullptr) {
            Serial.println("ERR DRIVE");
            return;
        }

        if (!motorsEnabled) {
            Serial.println("ERR MOTOR_OFF");
            return;
        }

        if (!imuReady || !imuCalibrated) {
            Serial.println("ERR IMU_NOT_READY");
            return;
        }

        const float velocityMps =
            atof(velocityString);

        const float pathHeadingDeg =
            atof(headingString);

        const float lateralErrorM =
            atof(lateralErrorString);

        /*
    DRIVE with zero velocity performs a planned stop.
    The normal motion profile handles deceleration.
    */
        if (fabsf(velocityMps) < 0.001f) {
            driveActive = false;

            setMotionTarget(
                0.0f,
                0.0f);

            Serial.println("ACK");
            return;
        }

        startDrive(
            velocityMps,
            pathHeadingDeg,
            lateralErrorM);

        Serial.println("ACK");

        return;
    }

    Serial.println("ERR CMD");
}



void readCommand() {
    while (Serial.available() > 0) {
        char receivedChar = static_cast<char>(Serial.read());

        // Ignore carriage return:
        if (receivedChar == '\r') {
            continue;
        }

        // Newline indicates end of command:
        if (receivedChar == '\n') {
            commandBuffer[commandIndex] = '\0';
            if (commandIndex > 0) {
                processCommand(commandBuffer);
            }
            commandIndex = 0;
            return;
        }

        // Store received characters:
        if (commandIndex < COMMAND_BUFFER_SIZE - 1) {
            commandBuffer[commandIndex] = receivedChar;
            commandIndex++;
        } else {
            // Command was too long.
            commandIndex = 0;
            Serial.println("ERR CMD_TOO_LONG");
        }
    }
}

constexpr uint32_t STATUS_PERIOD_US = 200000;
uint32_t lastStatusTimeUs = 0;

void updateStatusOutput() {
    const uint32_t nowUs = micros();
    if (nowUs - lastStatusTimeUs < STATUS_PERIOD_US) {
        return;
    }
    lastStatusTimeUs = nowUs;
    printStatus();
}



void setup() {
    Serial.begin(115200);
    delay(500);

    setupMotors();
    setupTimers();

    if (!setupImu()) {
        Serial.println("ERR IMU_NOT_FOUND");
    }

    Serial.println("READY");
}


void loop() {
    readCommand();
    updateImu();
    updateDriveControl();
    updateStatusOutput();
    updateMotionControl();
}