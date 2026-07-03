#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "soc/gpio_struct.h"

// --- Pin Definitions ---
constexpr uint8_t LEFT_STEP_PIN = 16;
constexpr uint8_t LEFT_DIR_PIN = 26;
constexpr uint8_t LEFT_EN_PIN = 25;

constexpr uint8_t RIGHT_STEP_PIN = 4;
constexpr uint8_t RIGHT_DIR_PIN = 13;
constexpr uint8_t RIGHT_EN_PIN = 14;

// --- IMU Definitions ---
constexpr uint8_t MPU6050_ADDRESS = 0x68;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_GYRO_ZOUT_H = 0x47;
constexpr float GYRO_SCALE_250DPS = 131.0f;

constexpr bool DRIVER_ENABLE_LEVEL = LOW;
constexpr bool DRIVER_DISABLE_LEVEL = HIGH;

constexpr uint32_t IMU_CALIBRATION_SAMPLES = 1000;
constexpr uint8_t IMU_CALIBRATION_DELAY_MS = 2;

// --- Kinematics & Movement Definitions ---
constexpr float WHEEL_DIAMETER_MM = 117.0f;
constexpr uint32_t STEPS_PER_REV = 20000;
constexpr float WHEEL_CIRCUMFERENCE_MM = PI * WHEEL_DIAMETER_MM;
constexpr float STEPS_PER_MM = STEPS_PER_REV / WHEEL_CIRCUMFERENCE_MM; 

constexpr float TARGET_DISTANCE_MM = 500.0f;
constexpr float PHASE1_DISTANCE_MM = TARGET_DISTANCE_MM * 0.70f; 

float BASE_SPEED_MM_S = 100.0f;      
constexpr float KP_HEADING = 2.5f;             
constexpr float KP_LATERAL = 3.0f;             // NEW: P-gain for lateral error (deg/s per mm of error)
constexpr float MAX_TURN_RATE_DEG = 60.0f;     

constexpr float WHEEL_BASE_MM = 355.0f; 
constexpr uint32_t TIMER_FREQ_HZ = 20000; 

// --- State Machine ---
enum RobotState {
    STATE_IDLE,
    STATE_PHASE1, 
    STATE_PHASE2, 
    STATE_FINISHED
};
volatile RobotState robotState = STATE_IDLE;

// --- Step Generation Variables ---
volatile uint32_t leftStepsPerSec = 0;
volatile uint32_t rightStepsPerSec = 0;
volatile bool leftDirForward = true;
volatile bool rightDirForward = true;

volatile uint32_t leftAccum = 0;
volatile uint32_t rightAccum = 0;

volatile int32_t totalLeftSteps = 0;
volatile int32_t totalRightSteps = 0;

volatile bool leftStepState = false;
volatile bool rightStepState = false;

hw_timer_t* stepTimer = NULL;

// --- IMU & Control Variables ---
float targetHeadingDeg = 0.0f;
float lockedHeadingDeg = 0.0f;
float lateralErrorMm = 0.0f; // NEW: Lateral error from tag X position

bool imuCalibrated = false;
float headingDeg = 0.0f;
float gyroRateDegPerSec = 0.0f;
float gyroBiasDegPerSec = 0.0f;
uint32_t lastUpdateMicros = 0;

// --- Forward Declarations ---
void IRAM_ATTR onStepTimer();
bool imuWriteRegister(uint8_t reg, uint8_t value);
bool imuReadGyroRaw(int16_t& raw);
float normalizeAngle(float angle);
bool imuBegin();
bool imuCalibrate();
void imuUpdate();
void resetImuHeading();
void stepper_begin();
void setWheelSpeeds(float leftSpeedMmPerSec, float rightSpeedMmPerSec);
void stopMotors();
void serialUpdate();

// --- IMU Functions ---
bool imuWriteRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool imuReadGyroRaw(int16_t& raw) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(REG_GYRO_ZOUT_H);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(MPU6050_ADDRESS, static_cast<uint8_t>(2)) != 2) return false;
    raw = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
    return true;
}

float normalizeAngle(float angle) {
    if (angle > 180.0f) return angle - 360.0f;
    else if (angle < -180.0f) return angle + 360.0f;
    return angle;
}

bool imuBegin() {
    Serial.println("imu_begin");
    Wire.begin();
    if (!imuWriteRegister(REG_PWR_MGMT_1, 0x00)) return false;
    delay(100);
    if (!imuWriteRegister(REG_GYRO_CONFIG, 0x00)) return false;
    delay(50);
    lastUpdateMicros = micros();
    return true;
}

bool imuCalibrate() {
    int32_t sum = 0;
    for (uint16_t i = 0; i < IMU_CALIBRATION_SAMPLES; ++i) {
        int16_t raw;
        if (!imuReadGyroRaw(raw)) return false;
        sum += raw;
        delay(IMU_CALIBRATION_DELAY_MS);
    }
    gyroBiasDegPerSec = (static_cast<float>(sum) / IMU_CALIBRATION_SAMPLES) / GYRO_SCALE_250DPS;
    headingDeg = 0.0f;
    lastUpdateMicros = micros();
    imuCalibrated = true;
    return true;
}

void resetImuHeading() {
    headingDeg = 0.0f;
    lastUpdateMicros = micros();
}

void imuUpdate() {
    if (!imuCalibrated) return;
    const uint32_t now = micros();
    const float dt = static_cast<float>(now - lastUpdateMicros) / 1000000.0f;
    lastUpdateMicros = now;
    if (dt <= 0.0f || dt > 0.1f) return;

    int16_t raw;
    if (!imuReadGyroRaw(raw)) return;
    gyroRateDegPerSec = (static_cast<float>(raw) / GYRO_SCALE_250DPS) - gyroBiasDegPerSec;
    headingDeg += gyroRateDegPerSec * dt;
    headingDeg = normalizeAngle(headingDeg);
}

// --- Stepper & Kinematics Functions ---
void IRAM_ATTR onStepTimer() {
    leftAccum += leftStepsPerSec;
    rightAccum += rightStepsPerSec;
    
    // Generate Left Steps
    while (leftAccum >= TIMER_FREQ_HZ) {
        leftAccum -= TIMER_FREQ_HZ;
        leftStepState = !leftStepState;
        
        // FIX: Only count steps on the rising edge (when state becomes true/HIGH)
        if (leftStepState) {
            GPIO.out_w1ts = (1 << LEFT_STEP_PIN); 
            if (leftDirForward) totalLeftSteps++; else totalLeftSteps--;
        } else {
            GPIO.out_w1tc = (1 << LEFT_STEP_PIN);
        }
    }
    
    // Generate Right Steps
    while (rightAccum >= TIMER_FREQ_HZ) {
        rightAccum -= TIMER_FREQ_HZ;
        rightStepState = !rightStepState;
        
        // FIX: Only count steps on the rising edge
        if (rightStepState) {
            GPIO.out_w1ts = (1 << RIGHT_STEP_PIN); 
            if (rightDirForward) totalRightSteps++; else totalRightSteps--;
        } else {
            GPIO.out_w1tc = (1 << RIGHT_STEP_PIN);
        }
    }
}

void stepper_begin() {
    pinMode(LEFT_STEP_PIN, OUTPUT); pinMode(LEFT_DIR_PIN, OUTPUT); pinMode(LEFT_EN_PIN, OUTPUT);
    pinMode(RIGHT_STEP_PIN, OUTPUT); pinMode(RIGHT_DIR_PIN, OUTPUT); pinMode(RIGHT_EN_PIN, OUTPUT);

    digitalWrite(LEFT_STEP_PIN, LOW); digitalWrite(RIGHT_STEP_PIN, LOW);
    digitalWrite(LEFT_EN_PIN, DRIVER_ENABLE_LEVEL); digitalWrite(RIGHT_EN_PIN, DRIVER_ENABLE_LEVEL);

    stepTimer = timerBegin(1000000); 
    timerAttachInterrupt(stepTimer, &onStepTimer);
    timerAlarm(stepTimer, (1000000 / TIMER_FREQ_HZ), true, 0); 
    timerStop(stepTimer);
}

void setWheelSpeeds(float leftSpeedMmPerSec, float rightSpeedMmPerSec) {
    leftDirForward = (leftSpeedMmPerSec >= 0);
    rightDirForward = (rightSpeedMmPerSec >= 0);
    
    digitalWrite(LEFT_DIR_PIN, leftDirForward ? HIGH : LOW);
    digitalWrite(RIGHT_DIR_PIN, rightDirForward ? HIGH : LOW);
    
    leftStepsPerSec = (uint32_t)(abs(leftSpeedMmPerSec) * STEPS_PER_MM);
    rightStepsPerSec = (uint32_t)(abs(rightSpeedMmPerSec) * STEPS_PER_MM);
}

void stopMotors() {
    timerStop(stepTimer);
    setWheelSpeeds(0, 0);
    digitalWrite(LEFT_STEP_PIN, LOW);
    digitalWrite(RIGHT_STEP_PIN, LOW);
}

void serialUpdate() {
    if (!Serial.available()) return;
    String command = Serial.readStringUntil('\n');
    command.trim();

    // CHANGED: HEAD now receives ABSOLUTE desired heading (map frame)
    if (command.startsWith("HEAD")) {
        targetHeadingDeg = command.substring(4).toFloat();
        targetHeadingDeg = normalizeAngle(targetHeadingDeg);
    }
    else if (command.startsWith("LAT")) {
        lateralErrorMm = command.substring(3).toFloat();
    }
    else if (command.startsWith("MOVE")) {
        if (robotState == STATE_PHASE1 || robotState == STATE_PHASE2) return;
        totalLeftSteps = 0; totalRightSteps = 0;
        leftAccum = 0; rightAccum = 0;
        robotState = STATE_PHASE1;
        timerStart(stepTimer);
    }
    else if (command.startsWith("STOP")) {
        robotState = STATE_IDLE;
        stopMotors();
    }
    else if (command.startsWith("SPD")) {
        float newSpeed = command.substring(3).toFloat();
        if (newSpeed > 0.0f && newSpeed < 1000.0f) {
            BASE_SPEED_MM_S = newSpeed;
        }
    }
}


void setup() {
    Serial.begin(115200);
    delay(500);
    
    imuBegin();
    Serial.println("Calibrating IMU... Keep robot still...");
    if (imuCalibrate()) Serial.println("IMU Calibrated");
    else Serial.println("IMU Calibration Failed");
    resetImuHeading();

    stepper_begin();
    Serial.println("Ready");
}

void loop() {
    serialUpdate();
    imuUpdate();

    float currentDistanceMm = ((float)totalLeftSteps + (float)totalRightSteps) / 2.0f / STEPS_PER_MM;

    if (robotState == STATE_PHASE1) {
        if (currentDistanceMm >= PHASE1_DISTANCE_MM) {
            robotState = STATE_PHASE2;
            lockedHeadingDeg = headingDeg;
        } else {
            // ESP32 computes heading error INTERNALLY from its IMU
            float errorHeading = normalizeAngle(targetHeadingDeg - headingDeg);
            float correctionHeading = errorHeading * KP_HEADING;

            // Lateral correction (positive lateral = tag to right = need to turn right = negative omega)
            float correctionLateral = -lateralErrorMm * KP_LATERAL;

            float totalCorrectionDegPerSec = constrain(
                correctionHeading + correctionLateral,
                -MAX_TURN_RATE_DEG, MAX_TURN_RATE_DEG
            );
            float correctionRadPerSec = totalCorrectionDegPerSec * PI / 180.0f;

            float vLeft = BASE_SPEED_MM_S - correctionRadPerSec * (WHEEL_BASE_MM / 2.0f);
            float vRight = BASE_SPEED_MM_S + correctionRadPerSec * (WHEEL_BASE_MM / 2.0f);

            setWheelSpeeds(vLeft, vRight);
        }
    }
    else if (robotState == STATE_PHASE2) {
        if (currentDistanceMm >= TARGET_DISTANCE_MM) {
            robotState = STATE_FINISHED;
            stopMotors();
        } else {
            // Phase 2: only correct heading error (ignore lateral)
            float errorHeading = normalizeAngle(lockedHeadingDeg - headingDeg);
            float totalCorrectionDegPerSec = constrain(
                errorHeading * KP_HEADING,
                -MAX_TURN_RATE_DEG, MAX_TURN_RATE_DEG
            );
            float correctionRadPerSec = totalCorrectionDegPerSec * PI / 180.0f;

            float vLeft = BASE_SPEED_MM_S - correctionRadPerSec * (WHEEL_BASE_MM / 2.0f);
            float vRight = BASE_SPEED_MM_S + correctionRadPerSec * (WHEEL_BASE_MM / 2.0f);

            setWheelSpeeds(vLeft, vRight);
        }
    }

    // Debug output (Python can still read this if needed, but doesn't have to)
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 100) {
        lastPrint = millis();
        Serial.printf("Dist=%.1f | Head=%.2f | Target=%.2f | Lat=%.1f | State=%d\n",
                      currentDistanceMm, headingDeg, targetHeadingDeg, lateralErrorMm, robotState);
    }
}
