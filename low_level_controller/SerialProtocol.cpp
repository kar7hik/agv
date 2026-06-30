#include "SerialProtocol.h"

namespace {
constexpr uint8_t NAVIGATION_PAYLOAD_SIZE = 17;

float readFloatLe(const uint8_t* data) {
    float value = 0.0f;
    memcpy(&value, data, sizeof(float));
    return value;
}

uint32_t readU32Le(const uint8_t* data) {
    return (static_cast<uint32_t>(data[0]) << 0) |
           (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) |
           (static_cast<uint32_t>(data[3]) << 24);
}

struct ImuCorrectionPayload {
    float correctionDeg;
};

struct StatusPayload {
    uint8_t mode;
    uint8_t imuInitialized;
    uint8_t imuCalibrated;
    uint8_t driversEnabled;
    float headingDeg;
    float gyroZDegPerSec;
    float leftHz;
    float rightHz;
    float baseHz;
    float steeringHz;
};

struct HeadingPayload {
    float headingDeg;
    float gyroZDegPerSec;
};
}

bool SerialProtocol::begin(HardwareSerial& serial,
                           MotionController* motionController,
                           ImuManager* imuManager,
                           MotorManager* motorManager) {
    if (motionController == nullptr || imuManager == nullptr || motorManager == nullptr) {
        return false;
    }

    _serial = &serial;
    _motionController = motionController;
    _imuManager = imuManager;
    _motorManager = motorManager;

    _serial->begin(Config::SERIAL_BAUD);
    resetRx();
    return true;
}

void SerialProtocol::update() {
    if (_serial == nullptr) {
        return;
    }

    while (_serial->available() > 0) {
        consumeByte(static_cast<uint8_t>(_serial->read()));
    }
}

void SerialProtocol::sendStatusIfDue() {
    const uint32_t now = millis();
    if ((now - _lastStatusMillis) >= Config::STATUS_PERIOD_MS) {
        _lastStatusMillis = now;
        sendStatus();
    }
}

void SerialProtocol::sendStatus() {
    if (_motionController == nullptr || _imuManager == nullptr || _motorManager == nullptr) {
        return;
    }

    StatusPayload payload;
    payload.mode = static_cast<uint8_t>(_motionController->getMode());
    payload.imuInitialized = _imuManager->isInitialized() ? 1 : 0;
    payload.imuCalibrated = _imuManager->isCalibrated() ? 1 : 0;
    payload.driversEnabled = _motorManager->driversEnabled() ? 1 : 0;
    payload.headingDeg = _imuManager->getHeadingDeg();
    payload.gyroZDegPerSec = _imuManager->getGyroZDegPerSec();
    payload.leftHz = _motorManager->getLeftFrequencyHz();
    payload.rightHz = _motorManager->getRightFrequencyHz();
    payload.baseHz = _motionController->getBaseFrequencyHz();
    payload.steeringHz = _motionController->getSteeringHz();

    sendPacket(PacketType::Status, reinterpret_cast<const uint8_t*>(&payload), sizeof(payload));
}

void SerialProtocol::sendHeading() {
    if (_imuManager == nullptr) {
        return;
    }

    HeadingPayload payload;
    payload.headingDeg = _imuManager->getHeadingDeg();
    payload.gyroZDegPerSec = _imuManager->getGyroZDegPerSec();
    sendPacket(PacketType::Heading, reinterpret_cast<const uint8_t*>(&payload), sizeof(payload));
}

void SerialProtocol::sendFault(uint8_t code) {
    sendPacket(PacketType::Fault, &code, 1);
}

void SerialProtocol::resetRx() {
    _rxState = RxState::WaitSof1;
    _rxLength = 0;
    _rxIndex = 0;
}

void SerialProtocol::consumeByte(uint8_t byte) {
    switch (_rxState) {
        case RxState::WaitSof1:
            if (byte == Config::PACKET_SOF_1) {
                _rxState = RxState::WaitSof2;
            }
            break;

        case RxState::WaitSof2:
            _rxState = (byte == Config::PACKET_SOF_2) ? RxState::ReadType : RxState::WaitSof1;
            break;

        case RxState::ReadType:
            _rxType = static_cast<PacketType>(byte);
            _rxState = RxState::ReadLength;
            break;

        case RxState::ReadLength:
            _rxLength = byte;
            if (_rxLength > Config::MAX_PAYLOAD_SIZE) {
                resetRx();
            } else if (_rxLength == 0) {
                _rxState = RxState::ReadChecksum;
            } else {
                _rxIndex = 0;
                _rxState = RxState::ReadPayload;
            }
            break;

        case RxState::ReadPayload:
            _rxPayload[_rxIndex++] = byte;
            if (_rxIndex >= _rxLength) {
                _rxState = RxState::ReadChecksum;
            }
            break;

        case RxState::ReadChecksum: {
            const uint8_t expected = checksum(_rxType, _rxLength, _rxPayload);
            if (byte == expected) {
                handlePacket(_rxType, _rxPayload, _rxLength);
            } else {
                sendFault(1); // checksum error
            }
            resetRx();
            break;
        }
    }
}

void SerialProtocol::handlePacket(PacketType type, const uint8_t* payload, uint8_t length) {
    switch (type) {
        case PacketType::Navigation: {
            if (length != NAVIGATION_PAYLOAD_SIZE) {
                sendFault(2);
                return;
            }

            NavigationCommand command;
            command.velocityMps = readFloatLe(&payload[0]);
            command.headingErrorDeg = readFloatLe(&payload[4]);
            command.lateralErrorM = readFloatLe(&payload[8]);
            command.flags = payload[12];
            command.timestampMs = readU32Le(&payload[13]);

            _motionController->setNavigationCommand(command);
            sendAck(type);
            break;
        }

        case PacketType::Stop:
            if (length != 0) {
                sendFault(2);
                return;
            }
            _motionController->stop();
            sendAck(type);
            break;

        case PacketType::ImuCalibrate:
            if (length != 0) {
                sendFault(2);
                return;
            }
            if (_imuManager->calibrate()) {
                sendPacket(PacketType::ImuCalibrated, nullptr, 0);
            } else {
                _motionController->setFault();
                sendFault(3);
            }
            break;

        case PacketType::EnableMotors:
            if (length != 0) {
                sendFault(2);
                return;
            }
            _motorManager->enableDrivers();
            _motionController->enterIdle();
            sendAck(type);
            break;

        case PacketType::DisableMotors:
            if (length != 0) {
                sendFault(2);
                return;
            }
            _motionController->stop();
            _motorManager->disableDrivers();
            sendAck(type);
            break;

        case PacketType::Ping:
            sendPacket(PacketType::Pong, nullptr, 0);
            break;

        case PacketType::ImuCorrection: {
            ImuCorrectionPayload correction;
            if (!readPayloadObject(payload, length, correction)) {
                sendFault(2);
                return;
            }
            _imuManager->applyHeadingCorrectionDeg(correction.correctionDeg);
            sendAck(type);
            break;
        }

        default:
            sendFault(4); // unsupported packet
            break;
    }
}

void SerialProtocol::sendAck(PacketType commandType) {
    const uint8_t payload = static_cast<uint8_t>(commandType);
    sendPacket(PacketType::Ack, &payload, 1);
}

void SerialProtocol::sendPacket(PacketType type, const uint8_t* payload, uint8_t length) {
    if (_serial == nullptr || length > Config::MAX_PAYLOAD_SIZE) {
        return;
    }

    _serial->write(Config::PACKET_SOF_1);
    _serial->write(Config::PACKET_SOF_2);
    _serial->write(static_cast<uint8_t>(type));
    _serial->write(length);

    for (uint8_t i = 0; i < length; ++i) {
        _serial->write(payload[i]);
    }

    _serial->write(checksum(type, length, payload));
}

uint8_t SerialProtocol::checksum(PacketType type, uint8_t length, const uint8_t* payload) {
    uint8_t sum = static_cast<uint8_t>(type) ^ length;
    for (uint8_t i = 0; i < length; ++i) {
        sum ^= payload[i];
    }
    return sum;
}
