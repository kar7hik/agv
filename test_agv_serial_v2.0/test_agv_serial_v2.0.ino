#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "soc/gpio_struct.h"

// ---------- Pins ----------
constexpr uint8_t LEFT_STEP_PIN = 16;
constexpr uint8_t LEFT_DIR_PIN = 26;
constexpr uint8_t LEFT_EN_PIN = 25;
constexpr uint8_t RIGHT_STEP_PIN = 4;
constexpr uint8_t RIGHT_DIR_PIN = 13;
constexpr uint8_t RIGHT_EN_PIN = 14;

// ---------- IMU (MPU6050) ----------
constexpr uint8_t MPU6050_ADDRESS = 0x68;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_GYRO_ZOUT_H = 0x47;
constexpr float GYRO_SCALE_250DPS = 131.0f;

constexpr bool DRIVER_ENABLE_LEVEL = LOW;
constexpr bool DRIVER_DISABLE_LEVEL = HIGH;
constexpr uint32_t IMU_CALIBRATION_SAMPLES = 1000;
constexpr uint8_t IMU_CALIBRATION_DELAY_MS = 2;

// ---------- Kinematics ----------
constexpr float WHEEL_DIAMETER_MM = 117.0f;
constexpr uint32_t STEPS_PER_REV = 20000;
constexpr float WHEEL_CIRCUMFERENCE_MM = PI * WHEEL_DIAMETER_MM;
constexpr float STEPS_PER_MM = STEPS_PER_REV / WHEEL_CIRCUMFERENCE_MM;

constexpr float WHEEL_BASE_MM = 355.0f;
constexpr float CORRECTION_DISTANCE_MM = 350.0f;       // fixed distance for atan-based steering
constexpr float MAX_SEGMENT_DISTANCE_MM = 10000.0f;    // safety cap only

float BASE_SPEED_MM_S = 100.0f;
constexpr float KP_HEADING = 2.5f;
constexpr float MAX_TURN_RATE_DEG = 60.0f;

constexpr uint32_t TIMER_FREQ_HZ = 20000;

float ROTATION_SPEED_DEG_S = 0.5 * BASE_SPEED_MM_S;    // Slower than forward speed
constexpr float HEADING_TOLERANCE_DEG = 2.0f;                    // Accept within 2 degrees

// ---------- State ----------
enum RobotState {
    STATE_IDLE,
    STATE_MOVING,
    STATE_ROTATING,
    STATE_FINISHED
};
volatile RobotState robotState = STATE_IDLE;

float targetRotationDeg = 0.0f;
float rotationSpeedDegPerSec = 0.0f;

// ---------- Step generation ----------
volatile uint32_t leftStepsPerSec = 0, rightStepsPerSec = 0;
volatile bool leftDirForward = true, rightDirForward = true;
volatile uint32_t leftAccum = 0, rightAccum = 0;
volatile int32_t totalLeftSteps = 0, totalRightSteps = 0;
volatile bool leftStepState = false, rightStepState = false;
hw_timer_t* stepTimer = NULL;

// ---------- IMU / control targets ----------
float targetHeadingDeg = 0.0f;
float lateralErrorMm = 0.0f;
bool imuCalibrated = false;
float headingDeg = 0.0f;
float gyroRateDegPerSec = 0.0f;
float gyroBiasDegPerSec = 0.0f;
uint32_t lastUpdateMicros = 0;

// ---------- Forward decls ----------
void IRAM_ATTR onStepTimer();
bool imuWriteRegister(uint8_t reg, uint8_t value);
bool imuReadGyroRaw(int16_t& raw);
float normalizeAngle(float angle);
bool imuBegin();
bool imuCalibrate();
void imuUpdate();
void resetImuHeading();
void stepper_begin();
void setWheelSpeeds(float vLeft_mm_s, float vRight_mm_s);
void stopMotors();
void serialUpdate();

// =====================================================================
// IMU
// =====================================================================
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
    if (angle < -180.0f) return angle + 360.0f;
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

// =====================================================================
// Stepper ISR
// =====================================================================
void IRAM_ATTR onStepTimer() {
    leftAccum += leftStepsPerSec;
    rightAccum += rightStepsPerSec;

    while (leftAccum >= TIMER_FREQ_HZ) {
        leftAccum -= TIMER_FREQ_HZ;
        leftStepState = !leftStepState;
        if (leftStepState) {
            GPIO.out_w1ts = (1 << LEFT_STEP_PIN);
            if (leftDirForward) totalLeftSteps++;
            else totalLeftSteps--;
        } else {
            GPIO.out_w1tc = (1 << LEFT_STEP_PIN);
        }
    }

    while (rightAccum >= TIMER_FREQ_HZ) {
        rightAccum -= TIMER_FREQ_HZ;
        rightStepState = !rightStepState;
        if (rightStepState) {
            GPIO.out_w1ts = (1 << RIGHT_STEP_PIN);
            if (rightDirForward) totalRightSteps++;
            else totalRightSteps--;
        } else {
            GPIO.out_w1tc = (1 << RIGHT_STEP_PIN);
        }
    }
}

void stepper_begin() {
    pinMode(LEFT_STEP_PIN, OUTPUT);
    pinMode(LEFT_DIR_PIN, OUTPUT);
    pinMode(LEFT_EN_PIN, OUTPUT);
    pinMode(RIGHT_STEP_PIN, OUTPUT);
    pinMode(RIGHT_DIR_PIN, OUTPUT);
    pinMode(RIGHT_EN_PIN, OUTPUT);

    digitalWrite(LEFT_STEP_PIN, LOW);
    digitalWrite(RIGHT_STEP_PIN, LOW);
    digitalWrite(LEFT_EN_PIN, DRIVER_ENABLE_LEVEL);
    digitalWrite(RIGHT_EN_PIN, DRIVER_ENABLE_LEVEL);

    stepTimer = timerBegin(1000000);
    timerAttachInterrupt(stepTimer, &onStepTimer);
    timerAlarm(stepTimer, (1000000 / TIMER_FREQ_HZ), true, 0);
    timerStop(stepTimer);
}

void setWheelSpeeds(float vLeft_mm_s, float vRight_mm_s) {
    leftDirForward = (vLeft_mm_s >= 0);
    rightDirForward = (vRight_mm_s >= 0);

    digitalWrite(LEFT_DIR_PIN, leftDirForward ? HIGH : LOW);
    digitalWrite(RIGHT_DIR_PIN, rightDirForward ? HIGH : LOW);

    leftStepsPerSec = (uint32_t)(fabsf(vLeft_mm_s) * STEPS_PER_MM);
    rightStepsPerSec = (uint32_t)(fabsf(vRight_mm_s) * STEPS_PER_MM);
}

void stopMotors() {
    timerStop(stepTimer);
    setWheelSpeeds(0, 0);
    digitalWrite(LEFT_STEP_PIN, LOW);
    digitalWrite(RIGHT_STEP_PIN, LOW);
}

// =====================================================================
// Serial
// =====================================================================
void serialUpdate() {
    if (!Serial.available()) return;
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("HEAD")) {
        targetHeadingDeg = normalizeAngle(command.substring(4).toFloat());
    } else if (command.startsWith("LAT")) {
        lateralErrorMm = command.substring(3).toFloat();
    } else if (command.startsWith("HCORR")) {
        headingDeg = normalizeAngle(command.substring(5).toFloat());
        lastUpdateMicros = micros();
        Serial.printf("[IMU] Corrected to %.2f deg\n", headingDeg);
    } else if (command.startsWith("MOVE")) {
        if (robotState == STATE_MOVING) return;
        totalLeftSteps = 0;
        totalRightSteps = 0;
        leftAccum = 0;
        rightAccum = 0;
        robotState = STATE_MOVING;
        timerStart(stepTimer);
        Serial.println("MOVE started");
    } else if (command.startsWith("STOP")) {
        robotState = STATE_IDLE;
        stopMotors();
        Serial.println("STOP");
    } else if (command.startsWith("SPD")) {
        float s = command.substring(3).toFloat();
        if (s > 0.0f && s < 1000.0f) BASE_SPEED_MM_S = s;
    } else if (command.startsWith("ROT")) {
        // ROT <target_heading_deg>
        targetRotationDeg = normalizeAngle(command.substring(4).toFloat());
        rotationSpeedDegPerSec = ROTATION_SPEED_DEG_S;
        robotState = STATE_ROTATING;
        Serial.printf("ROTATING to %.2f deg\n", targetRotationDeg);
    }
}

// =====================================================================
// Setup / Loop
// =====================================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    imuBegin();
    if (imuCalibrate()) Serial.println("IMU Calibrated");
    else Serial.println("IMU Calibration FAILED");
    resetImuHeading();

    stepper_begin();
    Serial.println("READY");
}

void loop() {
    serialUpdate();
    imuUpdate();

    float currentDistanceMm =
        ((float)totalLeftSteps + (float)totalRightSteps) / 2.0f / STEPS_PER_MM;

    if (robotState == STATE_ROTATING) {
    float headingError = normalizeAngle(targetRotationDeg - headingDeg);
    
    if (fabs(headingError) < HEADING_TOLERANCE_DEG) {
        // Rotation complete
        stopMotors();
        robotState = STATE_IDLE;
        Serial.println("Rotation complete");
    } else {
        // Rotate in place: left forward, right backward (or vice versa)
        float correctionRadPerSec = headingError * PI / 180.0f;
        correctionRadPerSec = constrain(correctionRadPerSec, 
                                        -rotationSpeedDegPerSec * PI / 180.0f,
                                        rotationSpeedDegPerSec * PI / 180.0f);
        
        float vLeft  = -correctionRadPerSec * (WHEEL_BASE_MM / 2.0f);
        float vRight =  correctionRadPerSec * (WHEEL_BASE_MM / 2.0f);
        
        setWheelSpeeds(vLeft, vRight);
    }
}

    if (robotState == STATE_MOVING) {
        // Safety cap (Python normally stops us first)
        if (currentDistanceMm >= MAX_SEGMENT_DISTANCE_MM) {
            robotState = STATE_FINISHED;
            stopMotors();
            Serial.println("Safety distance reached");
        } else {
            // ---- UNIFIED CONTROL ----
            // 1. Convert lateral error (mm) into a heading adjustment (deg)
            //    using the fixed correction distance.
            float lateralAngleDeg = atan2f(lateralErrorMm, CORRECTION_DISTANCE_MM)
                                    * 180.0f / PI;

            // 2. Combined target = desired heading + lateral adjustment
            float combinedTargetDeg = normalizeAngle(targetHeadingDeg + lateralAngleDeg);

            // 3. Single error term against IMU
            float totalErrorDeg = normalizeAngle(combinedTargetDeg - headingDeg);

            // 4. P-controller -> turn rate
            float correctionDegPerSec = constrain(
                totalErrorDeg * KP_HEADING,
                -MAX_TURN_RATE_DEG, MAX_TURN_RATE_DEG);
            float correctionRadPerSec = correctionDegPerSec * PI / 180.0f;

            // 5. Differential drive
            float vLeft = BASE_SPEED_MM_S - correctionRadPerSec * (WHEEL_BASE_MM / 2.0f);
            float vRight = BASE_SPEED_MM_S + correctionRadPerSec * (WHEEL_BASE_MM / 2.0f);

            setWheelSpeeds(vLeft, vRight);

            // Debug
            static uint32_t lastCtrlPrint = 0;
            if (millis() - lastCtrlPrint >= 200) {
                lastCtrlPrint = millis();
                Serial.printf("[CTRL] IMU=%6.2f  Target=%6.2f  LatAng=%5.2f  "
                              "CombTgt=%6.2f  Err=%6.2f  Omega=%5.2f deg/s\n",
                              headingDeg, targetHeadingDeg, lateralAngleDeg,
                              combinedTargetDeg, totalErrorDeg, correctionDegPerSec);
            }
        }
    }

    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 100) {
        lastPrint = millis();
        Serial.printf("Dist=%.1f | Head=%.2f | Target=%.2f | Lat=%.1f | State=%d\n",
                      currentDistanceMm, headingDeg, targetHeadingDeg,
                      lateralErrorMm, robotState);
    }
}