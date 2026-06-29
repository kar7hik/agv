#include <Arduino.h>
/* ===================== PIN CONFIG ===================== */

const int stepPin1 = 16;
const int dirPin1  = 26;
const int enPin1   = 25;

const int stepPin2 = 4;
const int dirPin2  = 13;
const int enPin2   = 14;

/* ===================== TIMERS ===================== */

hw_timer_t *leftTimer  = NULL;
hw_timer_t *rightTimer = NULL;

volatile bool leftStepState  = false;
volatile bool rightStepState = false;

/* ===================== ISR ===================== */

void IRAM_ATTR onLeftTimer(){
    leftStepState = !leftStepState;
    digitalWrite(stepPin1, leftStepState);
}

void IRAM_ATTR onRightTimer(){
    rightStepState = !rightStepState;
    digitalWrite(stepPin2, rightStepState);
}

/* ===================== MOTOR CONTROL ===================== */

void stopLeftMotor(){
    timerAlarmDisable(leftTimer);
    digitalWrite(stepPin1, LOW);
}

void stopRightMotor(){
    timerAlarmDisable(rightTimer);
    digitalWrite(stepPin2, LOW);
}

void stopAllMotors(){
    stopLeftMotor();
    stopRightMotor();
}

void setLeftMotor(float frequencyHz){
    if (frequencyHz == 0){
        stopLeftMotor();
        return;
    }

    if (frequencyHz > 0)
        digitalWrite(dirPin1, HIGH);
    else
        digitalWrite(dirPin1, LOW);

    frequencyHz = fabs(frequencyHz);

    uint32_t period_us = (uint32_t)(1000000.0 / (2.0 * frequencyHz));

    timerAlarmWrite(leftTimer, period_us, true);
    timerAlarmEnable(leftTimer);
}

void setRightMotor(float frequencyHz){
    if (frequencyHz == 0){
        stopRightMotor();
        return;
    }

    if (frequencyHz > 0)
        digitalWrite(dirPin2, HIGH);
    else
        digitalWrite(dirPin2, LOW);

    frequencyHz = fabs(frequencyHz);

    uint32_t period_us =
        (uint32_t)(1000000.0 / (2.0 * frequencyHz));

    timerAlarmWrite(rightTimer, period_us, true);
    timerAlarmEnable(rightTimer);
}
/* ===================== SETUP ===================== */

void setup(){
    Serial.begin(115200);

    pinMode(stepPin1, OUTPUT);
    pinMode(dirPin1, OUTPUT);
    pinMode(enPin1, OUTPUT);

    pinMode(stepPin2, OUTPUT);
    pinMode(dirPin2, OUTPUT);
    pinMode(enPin2, OUTPUT);

    digitalWrite(enPin1, LOW);
    digitalWrite(enPin2, LOW);

    /*
        80 MHz APB Clock
        Divider 80
        -> 1 MHz timer clock
        -> 1 tick = 1 us
    */

    leftTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(leftTimer, &onLeftTimer, true);
    timerAlarmWrite(leftTimer, 1000, true);
    timerAlarmDisable(leftTimer);

    rightTimer = timerBegin(1, 80, true);
    timerAttachInterrupt(rightTimer, &onRightTimer, true);
    timerAlarmWrite(rightTimer, 1000, true);
    timerAlarmDisable(rightTimer);
}

/* ===================== LOOP ===================== */

void loop(){
    if (!Serial.available())
        return;

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if(cmd == "s"){
        stopAllMotors();
        Serial.println("ACK STOP");
        return;
    }

    char command[10];
    float leftFreq = 0;
    float rightFreq = 0;

    int parsed = sscanf(cmd.c_str(), "%s %f %f", command, &leftFreq, &rightFreq);

    if (parsed == 3 && strcmp(command, "vel") == 0){
        setLeftMotor(leftFreq);
        setRightMotor(rightFreq);

        Serial.printf("ACK %.2f %.2f\n", leftFreq, rightFreq);
    }
}