#include <Arduino.h>
#include <Wire.h>

constexpr uint8_t LEFT_STEP_PIN = 16;
constexpr uint8_t LEFT_DIR_PIN = 26;
constexpr uint8_t LEFT_EN_PIN = 25;

constexpr uint8_t RIGHT_STEP_PIN = 4;
constexpr uint8_t RIGHT_DIR_PIN = 13;
constexpr uint8_t RIGHT_EN_PIN = 14;

constexpr uint8_t MPU6050_ADDRESS = 0x68;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_GYRO_ZOUT_H = 0x47;

constexpr float GYRO_SCALE_250DPS = 131.0f;

constexpr bool DRIVER_ENABLE_LEVEL = LOW;
constexpr bool DRIVER_DISABLE_LEVEL = HIGH;

constexpr float MOTOR_PULSES_PER_REVOLUTION = 20000.0f;
constexpr float WHEEL_DIAMETER_MM = 117.0f;
constexpr float WHEEL_CIRCUMFERENCE_MM = PI * WHEEL_DIAMETER_MM;

constexpr float TAG_DISTANCE_MM = 500.0f;
constexpr float CORRECTION_DISTANCE_MM = TAG_DISTANCE_MM * 0.70f;    // 350mm

constexpr float WHEEL_BASE_MM = 0.324f;

constexpr uint32_t IMU_CALIBRATION_SAMPLES = 1000;
constexpr uint8_t IMU_CALIBRATION_DELAY_MS = 2;

constexpr uint32_t INITIAL_TIMER_PERIOD_US = 1000;

volatile bool leftStepState = false;
volatile bool rightStepState = false;
volatile int32_t leftStepCount = 0;
volatile int32_t rightStepCount = 0;

constexpr float PULSES_PER_MM = MOTOR_PULSES_PER_REVOLUTION / (PI * WHEEL_CIRCUMFERENCE_MM);

float targetHeadingDeg = 0.0f;

bool headingControlActive = false;


bool imuCalibrated = false;
float headingDeg = 0.0f;
float gyroRateDegPerSec = 0.0f;
float gyroBiasDegPerSec = 0.0f;
uint32_t lastUpdateMicros = 0;

hw_timer_t* leftTimer = NULL;
hw_timer_t* rightTimer = NULL;

void IRAM_ATTR onLeftTimer();
void IRAM_ATTR onRightTimer();

bool enabled = false;

enum MotionState {
    IDLE,
    DRIVE_CORRECT,
    DRIVE_STRAIGHT
};

MotionState motionState = IDLE;


bool imuWriteRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(reg);
    Wire.write(value);

    return Wire.endTransmission() == 0;
}

bool imuReadGyroRaw(int16_t& raw) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(REG_GYRO_ZOUT_H);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom(MPU6050_ADDRESS, static_cast<uint8_t>(2)) != 2) {
        return false;
    }

    raw = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
    return true;
}


float normalizeAngle(float angle) {
    if (angle > 180.0f) {
        return angle - 360.0f;
    } else if (angle < -180.0f) {
        return angle + 360.0f;
    }
    return angle;
}

bool imuBegin() {
    Serial.println("imu_begin");
    Wire.begin();

    if (!imuWriteRegister(REG_PWR_MGMT_1, 0x00)) {
        Serial.println("imuWriteRegister failed - pwr mgmt");
        return false;
    }
    delay(100);

    if (!imuWriteRegister(REG_GYRO_CONFIG, 0x00)) {
        Serial.println("imuWriteRegister failed - gyro config");
        return false;
    }
    delay(50);

    lastUpdateMicros = micros();
    Serial.println("imu_begin done");
    return true;
}

bool imuCalibrate() {
    int32_t sum = 0;

    for (uint16_t i = 0; i < IMU_CALIBRATION_SAMPLES; ++i) {
        int16_t raw;

        if (!imuReadGyroRaw(raw)) {
            imuCalibrated = false;
            Serial.println("imuReadGyroRaw failed");
            return false;
        }

        sum += raw;
        delay(IMU_CALIBRATION_DELAY_MS);
    }

    const float averageRaw = static_cast<float>(sum) / IMU_CALIBRATION_SAMPLES;

    gyroBiasDegPerSec = averageRaw / GYRO_SCALE_250DPS;

    headingDeg = 0.0f;
    gyroRateDegPerSec = 0.0f;
    lastUpdateMicros = micros();
    imuCalibrated = true;

    Serial.println("imu_calibrate");
    return true;
}

void imuUpdate() {
    if (!imuCalibrated) {
        return;
    }

    const uint32_t now = micros();
    const float dt = static_cast<float>(now - lastUpdateMicros) / 1000000.0f;
    lastUpdateMicros = now;

    if (dt <= 0.0f || dt > 0.1f) {
        return;
    }

    int16_t raw;
    if (!imuReadGyroRaw(raw)) {
        return;
    }

    gyroRateDegPerSec = (static_cast<float>(raw) / GYRO_SCALE_250DPS) - gyroBiasDegPerSec;

    headingDeg += gyroRateDegPerSec * dt;
    headingDeg = normalizeAngle(headingDeg);
}


void resetImuHeading() {
    headingDeg = 0.0f;
    lastUpdateMicros = micros();
}

void stepper_begin() {
    Serial.println("stepper_begin");
    pinMode(LEFT_STEP_PIN, OUTPUT);
    pinMode(LEFT_DIR_PIN, OUTPUT);
    pinMode(LEFT_EN_PIN, OUTPUT);

    pinMode(RIGHT_STEP_PIN, OUTPUT);
    pinMode(RIGHT_DIR_PIN, OUTPUT);
    pinMode(RIGHT_EN_PIN, OUTPUT);

    digitalWrite(LEFT_STEP_PIN, LOW);
    digitalWrite(RIGHT_STEP_PIN, LOW);

    Serial.println("stepper_begin done");

    leftTimer = timerBegin(1000000);
    rightTimer = timerBegin(1000000);

    timerAttachInterrupt(leftTimer, &onLeftTimer);
    timerAttachInterrupt(rightTimer, &onRightTimer);

    timerAlarm(leftTimer,
               INITIAL_TIMER_PERIOD_US,
               true,
               0);

    timerAlarm(rightTimer,
               INITIAL_TIMER_PERIOD_US,
               true,
               0);
    timerStop(leftTimer);
    timerStop(rightTimer);
}

float getDistanceMM() {
    return ((leftStepCount + rightStepCount) * 0.5f) / PULSES_PER_MM;
}


void IRAM_ATTR onLeftTimer() {
    leftStepState = !leftStepState;
    digitalWrite(LEFT_STEP_PIN, leftStepState);

    if (leftStepState) {
        leftStepCount++;
    }
}

void IRAM_ATTR onRightTimer() {
    rightStepState = !rightStepState;
    digitalWrite(RIGHT_STEP_PIN, rightStepState);

    if (rightStepState) {
        rightStepCount++;
    }
}

void stepperEnable() {
    digitalWrite(LEFT_EN_PIN, DRIVER_ENABLE_LEVEL);
    digitalWrite(RIGHT_EN_PIN, DRIVER_ENABLE_LEVEL);
    enabled = true;
}

void stepperDiable() {
    digitalWrite(LEFT_EN_PIN, DRIVER_DISABLE_LEVEL);
    digitalWrite(RIGHT_EN_PIN, DRIVER_DISABLE_LEVEL);
    enabled = false;
}

void stepperStop() {
    timerStop(leftTimer);
    timerStop(rightTimer);

    digitalWrite(LEFT_STEP_PIN, LOW);
    digitalWrite(RIGHT_STEP_PIN, LOW);
}

void headingControllerUpdate() {
    if (!headingControlActive)
        return;

    float error = normalizeAngle(targetHeadingDeg - headingDeg);
    motionState = DRIVE_CORRECT;
    Serial.printf("Error = %.2f\n", error);

    if (fabs(error) < 0.2f) {
        stepperStop();
        headingControlActive = false;

        Serial.println("Heading Corrected");
        return;
    }

    if (error > 0.0f) {
        digitalWrite(LEFT_DIR_PIN, LOW);
        digitalWrite(RIGHT_DIR_PIN, HIGH);
    } else {
        digitalWrite(LEFT_DIR_PIN, HIGH);
        digitalWrite(RIGHT_DIR_PIN, LOW);
    }

    timerStart(leftTimer);
    timerStart(rightTimer);
}

void serialUpdate() {
    if (!Serial.available())
        return;

    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("HEAD")) {
        float headingError = command.substring(4).toFloat();
        targetHeadingDeg = normalizeAngle(headingDeg - headingError);
        headingControlActive = true;
        Serial.printf("Heading Error %.2f  Target %.2f\n",
                      headingError,
                      targetHeadingDeg);
    }
}

void setup() {
    Serial.begin(115200);
    imuBegin();
    Serial.println("Calibrating IMU...");
    if (imuCalibrate()) {
        Serial.println("IMU Calibrated");
    } else {
        Serial.println("IMU Calibration Failed");
    }

    resetImuHeading();

    delay(500);

    stepper_begin();
    stepperEnable();
}


void loop() {
    serialUpdate();

    imuUpdate();

    headingControllerUpdate();
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 50) {
        lastPrint = millis();

        Serial.printf(
            "Heading=%7.3f Target=%7.3f\n",
            headingDeg,
            targetHeadingDeg);
    }
}
