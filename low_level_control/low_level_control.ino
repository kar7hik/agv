#include <Arduino.h>
#include <string.h>
#include <Wire.h>
#include <stdlib.h>
#include <math.h>

// Serial
constexpr size_t COMMAND_BUFFER_SIZE = 128;
char commandBuffer[COMMAND_BUFFER_SIZE];
size_t commandIndex = 0;

// Telemetry:
constexpr uint32_t MIN_TELEMETRY_PERIOD_MS = 100;
constexpr uint32_t MAX_TELEMETRY_PERIOD_MS = 60000;

uint32_t telemetryPeriodUs = 0;
uint32_t lastTelemetryTimeUs = 0;


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



float driveVelocityMps = 0.0f;
float drivePathHeadingDeg = 0.0f;

float driveLateralErrorM = 0.0f;

// Current heading offset produced by the cubic trajectory.
float driveHeadingOffsetDeg = 0.0f;

uint32_t driveStartLeftPulseCount = 0;
uint32_t driveStartRightPulseCount = 0;

// Turn Control:
constexpr float TURN_HEADING_KP = 1.5f;
constexpr float DEFAULT_TURN_MAX_W_RAD_S = 0.08f;

constexpr float TURN_HEADING_TOLERANCE_DEG = 0.5f;
constexpr float TURN_STOPPED_W_RAD_S = 0.01f;
constexpr uint32_t TURN_SETTLE_TIME_US = 200000;



float turnTargetHeadingDeg = 0.0f;
float turnMaxAngularVelocityRadS = DEFAULT_TURN_MAX_W_RAD_S;
uint32_t turnSettledSinceUs = 0;

// Align Control:
constexpr float DEFAULT_ALIGN_MAX_W_RAD_S = 0.04f;

// Move Control:
constexpr float MOVE_HEADING_KP = 1.5f;
constexpr float MOVE_MAX_W_RAD_S = 0.08f;

constexpr float MOVE_DISTANCE_TOLERANCE_M = 0.002f;

float moveTargetDistanceM = 0.0f;
float movePathHeadingDeg = 0.0f;
float moveMaxVelocityMps = 0.0f;

int8_t moveDirection = 1;

uint32_t moveStartLeftPulseCount = 0;
uint32_t moveStartRightPulseCount = 0;

bool moveStopping = false;


enum class Operation {
    IDLE,
    DRIVE,
    TURN,
    ALIGN,
    MOVE,
    STOPPING
};

Operation activeOperation = Operation::IDLE;


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


const char *operationName(Operation operation) {
    switch (operation) {
        case Operation::IDLE:
            return "IDLE";
        case Operation::DRIVE:
            return "DRIVE";
        case Operation::TURN:
            return "TURN";
        case Operation::ALIGN:
            return "ALIGN";
        case Operation::MOVE:
            return "MOVE";
        case Operation::STOPPING:
            return "STOPPING";
    }

    return "UNKNOWN";
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
    activeOperation = Operation::IDLE;

    turnSettledSinceUs = 0;

    driveVelocityMps = 0.0f;
    drivePathHeadingDeg = 0.0f;
    driveLateralErrorM = 0.0f;
    driveHeadingOffsetDeg = 0.0f;

    moveTargetDistanceM = 0.0f;
    movePathHeadingDeg = 0.0f;
    moveMaxVelocityMps = 0.0f;
    moveDirection = 1;
    moveStopping = false;

    targetLinearVelocityMps = 0.0f;
    targetAngularVelocityRadS = 0.0f;

    currentLinearVelocityMps = 0.0f;
    currentAngularVelocityRadS = 0.0f;

    motionControlEnabled = false;

    stopMotion();
}

void startSmoothStop() {
    // Cancel ownership of DRIVE, TURN, ALIGN, and MOVE operations:
    // STOPPING is the new owner.
    activeOperation = Operation::STOPPING;

    // Prevent an old turn-settling timing from remaining active after a turn or alignment is interrupted:
    turnSettledSinceUs = 0;

    // Do not reset the current velocities:
    // The smooth stop will be applied to the current velocities.
    setMotionTarget(0.0f, 0.0f);
}

void updateMoveControl() {
    if (activeOperation != Operation::MOVE) {
        return;
    }

    // MOVE uses the IMU to hold an absolute heading.
    // Therefore, motors and calibrated IMU are required.
    if (!motorsEnabled || !imuReady || !imuCalibrated) {
        stopImmediate();
        Serial.println("ERR MOVE_ABORTED");
        return;
    }

    // Once the target distance has been reached, MOVE enters its stopping phase.
    if (moveStopping) {
        // UpdateMotionControl disables itself when both current velocities reached zero.
        if (motionControlEnabled) {
            return;
        }
        stopImmediate();
        Serial.println("DONE MOVE");
        return;
    }

    const float travelledM = moveDistanceTravelledM();
    const float remainingM = moveTargetDistanceM - travelledM;

    // The requested distance has been reached.
    // Begin the final controlled stop when the remaining distance enters the tolerance.
    // Keep Operation::MOVE active while the common motion controller ramps both velocities to zero.
    if (remainingM <= MOVE_DISTANCE_TOLERANCE_M) {
        moveStopping = true;
        setMotionTarget(0.0f, 0.0f);
        return;
    }

    // Continuous braking velocity:
    // v = sqrt(2 * a * remainingDistance)
    // As remaining distance decreases, the maximum safe target velocity also decreases.
    const float brakingVelocityMps = sqrt(2.0f * LINEAR_DECEL_MPS2 * remainingM);

    // Use either the safe braking velocity or the requested maximum velocity. Whichever is smaller.
    const float velocityMagnitudeMps = fminf(moveMaxVelocityMps, brakingVelocityMps);
    const float commandedVelocityMps = static_cast<float>(moveDirection) * velocityMagnitudeMps;

    /*
    Differential-drive wheel velocities are:

        left  = linear - angular * halfTrack
        right = linear + angular * halfTrack

    If angular velocity is too large compared with
    linear velocity, one wheel can reverse.
    */
    const float halfTrackM = TRACK_WIDTH_M * 0.5f;

    /*
    Use the current ramped linear velocity rather than
    only the requested target velocity.

    This also protects the robot during acceleration,
    when the target velocity may be high but the current
    velocity is still close to zero.
    */
    const float currentTranslationMagnitudeMps = fabsf(currentLinearVelocityMps);

    /*
    Keep the rotational wheel component below 80% of
    the current translational wheel velocity.

    Therefore, the slower wheel retains at least
    approximately 20% of the translation velocity.
    */
    const float maxAngularVelocityMps = currentTranslationMagnitudeMps * 0.8f / halfTrackM;


    /*
    Apply whichever angular limit is smaller:

    - general MOVE angular limit
    - translation-dependent limit
    */
    const float allowedAngularVelocityRadS = fminf(MOVE_MAX_W_RAD_S, maxAngularVelocityMps);



    // Calculate the heading correction using the restricted angular velocity:
    const float angularVelocityRadS = calculateHeadingAngularVelocityRadS(movePathHeadingDeg,
                                                                          MOVE_HEADING_KP,
                                                                          allowedAngularVelocityRadS);

    // updateMotionControl will ramp the actual linera and angular velocities to the target values.
    setMotionTarget(commandedVelocityMps, angularVelocityRadS);
}


void updateSmoothStop() {
    if (activeOperation != Operation::STOPPING) {
        return;
    }

    // updateMotionControl disables itself only after:
    // - target linear velocity is zero
    // - target angular velocity is zero
    // - current linear velocity reaches zero
    // - current angular velocity reaches zero
    if (motionControlEnabled) {
        return;
    }

    // The velocity controller has completed the stop.
    // Reset the remaining motion state and report completion:
    stopImmediate();

    Serial.println("DONE STOP");
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


float moveDistanceTravelledM() {
    const uint32_t leftPulseDelta = leftPulseCount - moveStartLeftPulseCount;
    const uint32_t rightPulseDelta = rightPulseCount - moveStartRightPulseCount;
    const float averagePulseDelta = (static_cast<float>(leftPulseDelta) + static_cast<float>(rightPulseDelta)) * 0.5f;

    return averagePulseDelta / PULSES_PER_METER;
}


float driveDistanceTravelledM() {
    const uint32_t leftPulseDelta = leftPulseCount - driveStartLeftPulseCount;
    const uint32_t rightPulseDelta = rightPulseCount - driveStartRightPulseCount;
    const float averagePulseDelta = (static_cast<float>(leftPulseDelta) + static_cast<float>(rightPulseDelta)) * 0.5f;

    return averagePulseDelta / PULSES_PER_METER;
}

float calculateDriveHeadingOffsetDeg(float progress) {
    const float s = clampFloat(progress, 0.0f, 1.0f);

    /*
    After the lateral-correction distance,
    hold the absolute path heading directly.
    */
    if (s >= 1.0f) {
        return 0.0f;
    }

    const float lateralSlope = (driveLateralErrorM / LATERAL_CORRECTION_DISTANCE_M) * (6.0f * s * s - 6.0f * s);
    const float headingOffsetDeg = atanf(lateralSlope) * 180.0f / PI_F;
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


void startMove(float distanceM, float pathHeadingDeg, float maxVelocityMps) {
    moveDirection = distanceM >= 0.0f ? 1 : -1;
    moveTargetDistanceM = fabsf(distanceM);
    movePathHeadingDeg = normalizeAngleDeg(pathHeadingDeg);
    moveMaxVelocityMps = clampFloat(fabsf(maxVelocityMps),
                                    0.0f,
                                    MAX_LINEAR_VELOCITY_MPS);

    moveStartLeftPulseCount = leftPulseCount;
    moveStartRightPulseCount = rightPulseCount;
    moveStopping = false;
    activeOperation = Operation::MOVE;
}

void startDrive(float velocityMps, float pathHeadingDeg, float lateralErrorM) {
    driveVelocityMps = clampFloat(velocityMps,
                                  -MAX_LINEAR_VELOCITY_MPS,
                                  MAX_LINEAR_VELOCITY_MPS);

    drivePathHeadingDeg = normalizeAngleDeg(pathHeadingDeg);
    driveLateralErrorM = lateralErrorM;

    /*
    At progress zero, the cubic desired heading should
    equal the robot's current corrected heading.
    */
    driveHeadingOffsetDeg = 0.0f;

    // Begin measuring correction distance from this point.
    driveStartLeftPulseCount = leftPulseCount;
    driveStartRightPulseCount = rightPulseCount;
    activeOperation = Operation::DRIVE;
}

void updateDriveControl() {
    if (activeOperation != Operation::DRIVE) {
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


void startHeadingTurn(float targetHeadingDeg, float maxAngularVelocityRadS, Operation operation) {
    turnTargetHeadingDeg = normalizeAngleDeg(targetHeadingDeg);
    turnMaxAngularVelocityRadS = clampFloat(fabsf(maxAngularVelocityRadS),
                                            0.0f,
                                            MAX_ANGULAR_VELOCITY_RAD_S);

    turnSettledSinceUs = 0;
    activeOperation = operation;
}



void startTurn(float targetHeadingDeg, float maxAngularVelocityRadS) {
    startHeadingTurn(targetHeadingDeg, maxAngularVelocityRadS, Operation::TURN);
}

void startAlign(float observedHeadingDeg, float targetHeadingDeg, float maxAngularVelocityRadS) {
    // Correct the IMU reference using the absolute heading measure from the apriltag.
    syncHeadingFromTag(normalizeAngleDeg(observedHeadingDeg));
    startHeadingTurn(targetHeadingDeg, maxAngularVelocityRadS, Operation::ALIGN);
}




void updateTurnControl() {
    const bool headingTurnActive = activeOperation == Operation::TURN || activeOperation == Operation::ALIGN;

    if (!headingTurnActive) {
        return;
    }

    // Turn cannot continue safely without
    // - Motors enabled
    // - IMU ready
    // - IMU calibrated
    if (!motorsEnabled || !imuReady || !imuCalibrated) {
        const Operation failedOperation = activeOperation;
        stopImmediate();

        if (failedOperation == Operation::ALIGN) {
            Serial.println("ERR ALIGN_ABORTED");
        } else {
            Serial.println("ERR TURN_ABORTED");
        }


        return;
    }

    const float headingErrorDeg = normalizeAngleDeg(turnTargetHeadingDeg - robotHeadingDeg());
    // Outside the target tolerance:
    // Continue rotating toward the target.
    if (fabsf(headingErrorDeg) > TURN_HEADING_TOLERANCE_DEG) {
        turnSettledSinceUs = 0;
        const float angularVelocityRadS = calculateHeadingAngularVelocityRadS(turnTargetHeadingDeg,
                                                                              TURN_HEADING_KP,
                                                                              turnMaxAngularVelocityRadS);
        setMotionTarget(0.0f, angularVelocityRadS);

        return;
    }

    // The heading is inside tolerance.
    // Command zero angular velocity and let updateMotionControl ramp it down.
    setMotionTarget(0.0f, 0.0f);

    const bool angularMotionStopped = fabsf(currentAngularVelocityRadS) <= TURN_STOPPED_W_RAD_S;

    if (!angularMotionStopped) {
        turnSettledSinceUs = 0;
        return;
    }

    const uint32_t nowUs = micros();

    // Start the settling timer when the robot first reaches both conditions:
    // - The heading is inside tolerance.
    // - The angular velocity nearly zero.
    if (turnSettledSinceUs == 0) {
        turnSettledSinceUs = nowUs;
        return;
    }

    // Continue waiting until the robot has remained settled for the required duration.
    if (nowUs - turnSettledSinceUs < TURN_SETTLE_TIME_US) {
        return;
    }

    const Operation completedOperation = activeOperation;
    stopImmediate();

    if (completedOperation == Operation::ALIGN) {
        Serial.println("DONE ALIGN");
    } else {
        Serial.println("DONE TURN");
    }
}

void printStatus() {
    Serial.print("STATUS MOTOR=");
    Serial.print(motorsEnabled ? "ON" : "OFF");

    Serial.print(" IMU=");
    Serial.print(imuReady ? "READY" : "ERROR");

    Serial.print(" CAL=");
    Serial.print(imuCalibrated ? "YES" : "NO");

    Serial.print(" MODE=");
    Serial.print(operationName(activeOperation));

    Serial.print(" RAW=");
    Serial.print(imuHeadingDeg, 2);

    Serial.print(" OFF=");
    Serial.print(headingCorrectionDeg, 2);

    Serial.print(" HDG=");
    Serial.print(robotHeadingDeg(), 2);

    const bool headingTurnActive = activeOperation == Operation::TURN || activeOperation == Operation::ALIGN;

    if (headingTurnActive) {
        const float headingErrorDeg = normalizeAngleDeg(turnTargetHeadingDeg - robotHeadingDeg());
        Serial.print(" TTGT=");
        Serial.print(turnTargetHeadingDeg, 2);

        Serial.print(" TERR=");
        Serial.print(headingErrorDeg, 2);

        Serial.print(" TMAXW=");
        Serial.print(turnMaxAngularVelocityRadS, 3);
    }

    if (activeOperation == Operation::DRIVE) {
        const float distanceM = driveDistanceTravelledM();
        const float progress = clampFloat(distanceM / LATERAL_CORRECTION_DISTANCE_M,
                                          0.0f,
                                          1.0f);

        Serial.print(" DIST=");
        Serial.print(distanceM, 2);

        Serial.print(" PROGRESS=");
        Serial.print(progress, 2);

        Serial.print(" DPHASE=");
        Serial.print(progress < 1.0f ? "CORRECT" : "HOLD");

        Serial.print(" PATH=");
        Serial.print(drivePathHeadingDeg, 2);

        Serial.print(" LAT=");
        Serial.print(driveLateralErrorM, 4);

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

    if (!strcmp(command, "TELEMETRY")) {
        char *periodString = strtok(nullptr, " ");

        if (periodString == nullptr) {
            Serial.println("ERR TELEMETRY");
            return;
        }

        // Convert the input for valid integer
        char *endPointer = nullptr;
        const long periodMs = strtol(periodString, &endPointer, 10);

        const bool invalidNumber = endPointer == periodString || *endPointer != '\0';

        if (invalidNumber) {
            Serial.println("ERR TELEMETRY");
            return;
        }

        // Zero disables automatic telemetry.
        if (periodMs == 0) {
            telemetryPeriodUs = 0;
            Serial.println("ACK");
            return;
        }

        // Rejects negative periods, excessively frequent output and excessively long periods.
        if (periodMs < static_cast<long>(MIN_TELEMETRY_PERIOD_MS) || periodMs > static_cast<long>(MAX_TELEMETRY_PERIOD_MS)) {
            Serial.println("ERR TELEMETRY_PERIOD");
            return;
        }

        telemetryPeriodUs = static_cast<uint32_t>(periodMs) * 1000UL;

        // Restart timing from changes requested
        lastTelemetryTimeUs = micros();

        Serial.println("ACK");
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

    if (!strcmp(command, "STOP_SMOOTH")) {
        if (activeOperation == Operation::STOPPING) {
            Serial.println("ERR BUSY");
            return;
        }

        // If the motors are disabled, or the robot is already completely idle,
        // there is no ramped motion to complete.
        const bool alreadyStopped = activeOperation == Operation::IDLE && !motionControlEnabled;

        if (!motorsEnabled || alreadyStopped) {
            stopImmediate();
            Serial.println("ACK");
            Serial.println("DONE STOP");
            return;
        }
        startSmoothStop();
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

        const float velocityMps = atof(velocityString);
        const float pathHeadingDeg = atof(headingString);
        const float lateralErrorM = atof(lateralErrorString);

        /*  
        DRIVE is never used as a stopping command.
        STOP: Immediate stop
        STOP_SMOOTH: Smooth stop
        */
        if (velocityMps <= 0.001f) {
            Serial.println("ERR DRIVE_SPEED");
            return;
        }

        const bool startingDrive = activeOperation == Operation::IDLE && !motionControlEnabled;
        const bool updatingDrive = activeOperation == Operation::DRIVE;

        if (!startingDrive && !updatingDrive) {
            Serial.println("ERR BUSY");
            return;
        }

        startDrive(velocityMps, pathHeadingDeg, lateralErrorM);

        Serial.println("ACK");

        return;
    }

    if (!strcmp(command, "MOVE")) {
        char *distanceString = strtok(nullptr, " ");
        char *headingString = strtok(nullptr, " ");
        char *velocityString = strtok(nullptr, " ");

        if (distanceString == nullptr
            || headingString == nullptr
            || velocityString == nullptr) {
            Serial.println("ERR MOVE");
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

        // Do not begin a new MOVE while another high-level or low-level motion is active.
        if (activeOperation != Operation::IDLE || motionControlEnabled) {
            Serial.println("ERR BUSY");
            return;
        }

        const float distanceM = atof(distanceString);
        const float pathHeadingDeg = atof(headingString);
        const float maxVelocityMps = atof(velocityString);

        if (maxVelocityMps <= 0.0f) {
            Serial.println("ERR MOVE_SPEED");
            return;
        }

        // A negligible movement is accepted and completed without starting the motors
        if (fabsf(distanceM) <= MOVE_DISTANCE_TOLERANCE_M) {
            Serial.println("ACK");
            Serial.println("DONE MOVE");
            return;
        }

        startMove(distanceM, pathHeadingDeg, maxVelocityMps);

        Serial.println("ACK");
        return;
    }

    if (!strcmp(command, "TURN")) {
        char *headingString = strtok(nullptr, " ");
        char *maxAngluarVelocityString = strtok(nullptr, " ");

        if (headingString == nullptr) {
            Serial.println("ERR TURN");
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

        if (activeOperation != Operation::IDLE) {
            Serial.println("ERR BUSY");
            return;
        }

        const float targetHeadingDeg = atof(headingString);
        float maxAngularVelocityRadS = DEFAULT_TURN_MAX_W_RAD_S;

        if (maxAngluarVelocityString != nullptr) {
            maxAngularVelocityRadS = atof(maxAngluarVelocityString);
        }

        if (maxAngularVelocityRadS <= 0.0f) {
            Serial.println("ERR TURN_SPEED");
            return;
        }

        startTurn(targetHeadingDeg, maxAngularVelocityRadS);

        Serial.println("ACK");


        return;
    }

    if (!strcmp(command, "ALIGN")) {
        char *observedHeadingString = strtok(nullptr, " ");
        char *targetHeadingString = strtok(nullptr, " ");
        char *maxAngularVelocityString = strtok(nullptr, " ");

        if (observedHeadingString == nullptr || targetHeadingString == nullptr) {
            Serial.println("ERR ALIGN");
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

        if (activeOperation != Operation::IDLE) {
            Serial.println("ERR BUSY");
            return;
        }

        const float observedHeadingDeg = atof(observedHeadingString);
        const float targetHeadingDeg = atof(targetHeadingString);
        float maxAngularVelocityRadS = DEFAULT_ALIGN_MAX_W_RAD_S;

        if (maxAngularVelocityString != nullptr) {
            maxAngularVelocityRadS = atof(maxAngularVelocityString);
        }

        if (maxAngularVelocityRadS <= 0.0f) {
            Serial.println("ERR ALIGN_SPEED");
            return;
        }

        startAlign(observedHeadingDeg, targetHeadingDeg, maxAngularVelocityRadS);

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


void updateTelemetryOutput() {
    // A zero period means automatic telemetry is disabled.
    if (telemetryPeriodUs == 0) {
        return;
    }

    const uint32_t nowUs = micros();
    if (nowUs - lastTelemetryTimeUs < telemetryPeriodUs) {
        return;
    }

    lastTelemetryTimeUs = nowUs;
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

    updateTurnControl();
    updateDriveControl();
    updateMoveControl();

    updateMotionControl();
    updateSmoothStop();

    updateTelemetryOutput();
}