#pragma once

#include <Arduino.h>
#include "Config.h"
#include "MotionController.h"
#include "ImuManager.h"
#include "MotorManager.h"

enum class PacketType : uint8_t {
    Navigation = 0x01,
    Stop = 0x02,
    ImuCalibrate = 0x03,
    EnableMotors = 0x04,
    DisableMotors = 0x05,
    Ping = 0x06,
    ImuCorrection = 0x07,

    Ack = 0x80,
    ImuCalibrated = 0x81,
    Fault = 0x82,
    Heading = 0x83,
    Status = 0x84,
    Pong = 0x85
};

class SerialProtocol {
public:
    bool begin(HardwareSerial& serial,
               MotionController* motionController,
               ImuManager* imuManager,
               MotorManager* motorManager);

    void update();
    void sendStatusIfDue();
    void sendStatus();
    void sendHeading();
    void sendFault(uint8_t code);

private:
    enum class RxState : uint8_t {
        WaitSof1,
        WaitSof2,
        ReadType,
        ReadLength,
        ReadPayload,
        ReadChecksum
    };

    void resetRx();
    void consumeByte(uint8_t byte);
    void handlePacket(PacketType type, const uint8_t* payload, uint8_t length);
    void sendAck(PacketType commandType);
    void sendPacket(PacketType type, const uint8_t* payload, uint8_t length);

    static uint8_t checksum(PacketType type, uint8_t length, const uint8_t* payload);

    template<typename T>
    static bool readPayloadObject(const uint8_t* payload, uint8_t length, T& out) {
        if (length != sizeof(T)) {
            return false;
        }
        memcpy(&out, payload, sizeof(T));
        return true;
    }

    HardwareSerial* _serial = nullptr;
    MotionController* _motionController = nullptr;
    ImuManager* _imuManager = nullptr;
    MotorManager* _motorManager = nullptr;

    RxState _rxState = RxState::WaitSof1;
    PacketType _rxType = PacketType::Ping;
    uint8_t _rxLength = 0;
    uint8_t _rxIndex = 0;
    uint8_t _rxPayload[Config::MAX_PAYLOAD_SIZE] = {0};

    uint32_t _lastStatusMillis = 0;
};
