#ifndef STSSERVO_DRIVER_HPP
#define STSSERVO_DRIVER_HPP

#include <cstdint>
#include <cstddef>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>

// Linux-friendly Arduino shims
using byte = uint8_t;

inline void delay(unsigned long ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
inline void delayMicroseconds(unsigned int us) {
    std::this_thread::sleep_for(std::chrono::microseconds(us));
}
// Direction pin shims (no-op on Linux)
constexpr int OUTPUT = 1;
constexpr int HIGH = 1;
constexpr int LOW = 0;
inline void pinMode(byte /*pin*/, int /*mode*/) {}
inline void digitalWrite(byte /*pin*/, int /*value*/) {}

// Token name used in the original driver. In Linux, define a class with the same
// method signatures and use `#define HardwareSerial YourSerialClass` before including
// this header, or simply use a class named `HardwareSerial`.

namespace STSRegisters {
    static constexpr byte FIRMWARE_MAJOR         = 0x00;
    static constexpr byte FIRMWARE_MINOR         = 0x01;
    static constexpr byte SERVO_MAJOR            = 0x03;
    static constexpr byte SERVO_MINOR            = 0x04;
    static constexpr byte ID                     = 0x05;
    static constexpr byte BAUDRATE               = 0x06;
    static constexpr byte RESPONSE_DELAY         = 0x07;
    static constexpr byte RESPONSE_STATUS_LEVEL  = 0x08;
    static constexpr byte MINIMUM_ANGLE          = 0x09;
    static constexpr byte MAXIMUM_ANGLE          = 0x0B;
    static constexpr byte MAXIMUM_TEMPERATURE    = 0x0D;
    static constexpr byte MAXIMUM_VOLTAGE        = 0x0E;
    static constexpr byte MINIMUM_VOLTAGE        = 0x0F;
    static constexpr byte MAXIMUM_TORQUE         = 0x10;
    static constexpr byte UNLOADING_CONDITION    = 0x13;
    static constexpr byte LED_ALARM_CONDITION    = 0x14;
    static constexpr byte POS_PROPORTIONAL_GAIN  = 0x15;
    static constexpr byte POS_DERIVATIVE_GAIN    = 0x16;
    static constexpr byte POS_INTEGRAL_GAIN      = 0x17;
    static constexpr byte MINIMUM_STARTUP_FORCE  = 0x18;
    static constexpr byte CK_INSENSITIVE_AREA    = 0x1A;
    static constexpr byte CCK_INSENSITIVE_AREA   = 0x1B;
    static constexpr byte CURRENT_PROTECTION_TH  = 0x1C;
    static constexpr byte ANGULAR_RESOLUTION     = 0x1E;
    static constexpr byte POSITION_CORRECTION    = 0x1F;
    static constexpr byte OPERATION_MODE         = 0x21;
    static constexpr byte TORQUE_PROTECTION_TH   = 0x22;
    static constexpr byte TORQUE_PROTECTION_TIME = 0x23;
    static constexpr byte OVERLOAD_TORQUE        = 0x24;
    static constexpr byte SPEED_PROPORTIONAL_GAIN= 0x25;
    static constexpr byte OVERCURRENT_TIME       = 0x26;
    static constexpr byte SPEED_INTEGRAL_GAIN    = 0x27;
    static constexpr byte TORQUE_SWITCH          = 0x28;
    static constexpr byte TARGET_ACCELERATION    = 0x29;
    static constexpr byte TARGET_POSITION        = 0x2A;
    static constexpr byte RUNNING_TIME           = 0x2C;
    static constexpr byte RUNNING_SPEED          = 0x2E;
    static constexpr byte TORQUE_LIMIT           = 0x30;
    static constexpr byte WRITE_LOCK             = 0x37;
    static constexpr byte CURRENT_POSITION       = 0x38;
    static constexpr byte CURRENT_SPEED          = 0x3A;
    static constexpr byte CURRENT_DRIVE_VOLTAGE  = 0x3C;
    static constexpr byte CURRENT_VOLTAGE        = 0x3E;
    static constexpr byte CURRENT_TEMPERATURE    = 0x3F;
    static constexpr byte ASYNCHRONOUS_WRITE_ST  = 0x40;
    static constexpr byte STATUS                 = 0x41;
    static constexpr byte MOVING_STATUS          = 0x42;
    static constexpr byte CURRENT_CURRENT        = 0x45;
}

enum STSMode { POSITION = 0, VELOCITY = 1, STEP = 3 };

enum ServoType { UNKNOWN = 0, STS = 1, SCS = 2 };

namespace instruction {
    static constexpr byte PING_      = 0x01;
    static constexpr byte READ       = 0x02;
    static constexpr byte WRITE      = 0x03;
    static constexpr byte REGWRITE   = 0x04;
    static constexpr byte ACTION     = 0x05;
    static constexpr byte SYNCWRITE  = 0x83;
    static constexpr byte RESET      = 0x06;
}

class STSServoDriver {
public:
    STSServoDriver() : port_(nullptr), dirPin_(0) {
        for (int i = 0; i < 256; ++i) servoType_[i] = ServoType::UNKNOWN;
    }

    bool init(byte const &dirPin, HardwareSerial *serialPort = nullptr, long const &baudRate = 1000000) {
        if (serialPort == nullptr) return false; // Linux: explicit serial required
        port_ = serialPort;
        port_->begin(baudRate);
        port_->setTimeout(2);
        dirPin_ = dirPin;
        if (dirPin_ < 255) {
            pinMode(dirPin_, OUTPUT);
        }
        for (int i = 0; i < 256; i++) servoType_[i] = ServoType::UNKNOWN;
        for (byte i = 0; i < 0xFE; i++)
            if (ping(i)) return true;
        return false;
    }
    bool init(HardwareSerial *serialPort = nullptr, long const &baudRate = 1000000) {
        return this->init(255, serialPort, baudRate);
    }

    bool ping(byte const &servoId) {
        byte response[1] = {0xFF};
        int send = sendMessage(servoId, instruction::PING_, 0, response);
        if (send != 6) return false;
        int rd = receiveMessage(servoId, 1, response);
        if (rd < 0) return false;
        return response[0] == 0x00;
    }

    bool setId(byte const &oldServoId, byte const &newServoId) {
        if (servoType_[oldServoId] == ServoType::UNKNOWN) {
            determineServoType(oldServoId);
        }
        if (oldServoId >= 0xFE || newServoId >= 0xFE) return false;
        if (ping(newServoId)) return false;
        unsigned char lockRegister = STSRegisters::WRITE_LOCK;
        if (servoType_[oldServoId] == ServoType::SCS) {
            lockRegister = STSRegisters::TORQUE_LIMIT;
        }
        if (!writeRegister(oldServoId, lockRegister, 0)) return false;
        delay(5);
        if (!writeRegister(oldServoId, STSRegisters::ID, newServoId)) return false;
        delay(5);
        if (!writeRegister(newServoId, lockRegister, 1)) return false;
        bool hasPing = false;
        int nIter = 0;
        while (!hasPing && nIter < 10) {
            delay(50);
            hasPing = ping(newServoId);
            ++nIter;
        }
        if (hasPing) {
            servoType_[newServoId] = servoType_[oldServoId];
            servoType_[oldServoId] = ServoType::UNKNOWN;
        }
        return hasPing;
    }

    bool setPositionOffset(byte const &servoId, int const &positionOffset) {
        if (!writeRegister(servoId, STSRegisters::WRITE_LOCK, 0)) return false;
        if (!writeTwoBytesRegister(servoId, STSRegisters::POSITION_CORRECTION, positionOffset)) return false;
        if (!writeRegister(servoId, STSRegisters::WRITE_LOCK, 1)) return false;
        return true;
    }

    int getCurrentPosition(byte const &servoId) { return readTwoBytesRegister(servoId, STSRegisters::CURRENT_POSITION); }
    int getCurrentSpeed(byte const &servoId)    { return readTwoBytesRegister(servoId, STSRegisters::CURRENT_SPEED); }
    int getCurrentTemperature(byte const &servoId) { return readTwoBytesRegister(servoId, STSRegisters::CURRENT_TEMPERATURE); }

    float getCurrentCurrent(byte const &servoId) {
        int16_t current = readTwoBytesRegister(servoId, STSRegisters::CURRENT_CURRENT);
        return current * 0.0065f;
    }

    bool isMoving(byte const &servoId) {
        byte const result = readRegister(servoId, STSRegisters::MOVING_STATUS);
        return result > 0;
    }

    bool setTargetPosition(byte const &servoId, int const &position, int const &speed = 4095, bool const &asynchronous = false) {
        byte params[6] = {0, 0, 0, 0, 0, 0};
        convertIntToBytes(servoId, position, &params[0]);
        convertIntToBytes(servoId, speed, &params[4]);
        return writeRegisters(servoId, STSRegisters::TARGET_POSITION, sizeof(params), params, asynchronous);
    }

    bool setTargetVelocity(byte const &servoId, int const &velocity, bool const &asynchronous = false) {
        return writeTwoBytesRegister(servoId, STSRegisters::RUNNING_SPEED, velocity, asynchronous);
    }

    bool setTargetAcceleration(byte const &servoId, byte const &acceleration, bool const &asynchronous = false) {
        return writeRegister(servoId, STSRegisters::TARGET_ACCELERATION, acceleration, asynchronous);
    }

    bool setMode(unsigned char const &servoId, STSMode const &mode) {
        return writeRegister(servoId, STSRegisters::OPERATION_MODE, static_cast<unsigned char>(mode));
    }

    bool trigerAction() {
        byte noParam = 0;
        int send = sendMessage(0xFE, instruction::ACTION, 0, &noParam);
        return send == 6;
    }

    bool writeRegister(byte const &servoId, byte const &registerId, byte const &value, bool const &asynchronous = false) {
        return writeRegisters(servoId, registerId, 1, &value, asynchronous);
    }

    bool writeTwoBytesRegister(byte const &servoId, byte const &registerId, int16_t const &value, bool const &asynchronous = false) {
        byte params[2] = {0, 0};
        convertIntToBytes(servoId, value, params);
        return writeRegisters(servoId, registerId, 2, params, asynchronous);
    }

    byte readRegister(byte const &servoId, byte const &registerId) {
        byte result = 0;
        int rc = readRegisters(servoId, registerId, 1, &result);
        if (rc < 0) return 0;
        return result;
    }

    int16_t readTwoBytesRegister(byte const &servoId, byte const &registerId) {
        if (servoType_[servoId] == ServoType::UNKNOWN) {
            determineServoType(servoId);
        }
        unsigned char result[2] = {0, 0};
        int16_t value = 0;
        int16_t signedValue = 0;
        int rc = readRegisters(servoId, registerId, 2, result);
        if (rc < 0) return 0;
        switch (servoType_[servoId]) {
            case ServoType::SCS:
                value = static_cast<int16_t>(result[1] + (result[0] << 8));
                signedValue = value & ~0x8000;
                if (value & 0x8000) signedValue = -signedValue;
                return signedValue;
            case ServoType::STS:
                value = static_cast<int16_t>(result[0] + (result[1] << 8));
                signedValue = value & ~0x8000;
                if (value & 0x8000) signedValue = -signedValue;
                return signedValue;
            default:
                return 0;
        }
    }

    void setTargetPositions(byte const &numberOfServos, const byte servoIds[], const int positions[], const int speeds[]) {
        port_->write((uint8_t)0xFF);
        port_->write((uint8_t)0xFF);
        port_->write((uint8_t)0xFE);
        port_->write((uint8_t)(numberOfServos * 7 + 4));
        port_->write((uint8_t)instruction::SYNCWRITE);
        port_->write((uint8_t)STSRegisters::TARGET_POSITION);
        port_->write((uint8_t)6);
        byte checksum = 0xFE + numberOfServos * 7 + 4 + instruction::SYNCWRITE + STSRegisters::TARGET_POSITION + 6;
        for (int index = 0; index < numberOfServos; index++) {
            checksum += servoIds[index];
            port_->write(servoIds[index]);
            byte intAsByte[2];
            convertIntToBytes(servoIds[index], positions[index], intAsByte);
            sendAndUpdateChecksum(intAsByte, checksum);
            port_->write((uint8_t)0);
            port_->write((uint8_t)0);
            convertIntToBytes(servoIds[index], speeds[index], intAsByte);
            sendAndUpdateChecksum(intAsByte, checksum);
        }
        port_->write((uint8_t)(~checksum));
    }

private:
    int sendMessage(byte const &servoId, byte const &commandID, byte const &paramLength, byte *parameters) {
        std::vector<byte> message(6 + paramLength);
        byte checksum = servoId + paramLength + 2 + commandID;
        message[0] = 0xFF;
        message[1] = 0xFF;
        message[2] = servoId;
        message[3] = paramLength + 2;
        message[4] = commandID;
        for (int i = 0; i < paramLength; i++) {
            message[5 + i] = parameters[i];
            checksum += parameters[i];
        }
        message[5 + paramLength] = (byte)~checksum;
        if (this->dirPin_ < 255) {
            digitalWrite(dirPin_, HIGH);
        }
        int ret = port_->write(message.data(), message.size());
        if (this->dirPin_ < 255) {
            digitalWrite(dirPin_, LOW);
        }
        delayMicroseconds(200);
        return ret;
    }

    bool writeRegisters(byte const &servoId, byte const &startRegister, byte const &writeLength, byte const *parameters, bool const &asynchronous = false) {
        std::vector<byte> param(writeLength + 1);
        param[0] = startRegister;
        for (int i = 0; i < writeLength; i++) param[i + 1] = parameters[i];
        int rc = sendMessage(servoId, asynchronous ? instruction::REGWRITE : instruction::WRITE, writeLength + 1, param.data());
        return rc == writeLength + 7;
    }

    int readRegisters(byte const &servoId, byte const &startRegister, byte const &readLength, byte *outputBuffer) {
        byte readParam[2] = {startRegister, readLength};
        while (port_->read() != -1) { /* flush */ }
        int send = sendMessage(servoId, instruction::READ, 2, readParam);
        if (send != 8) return -1;
        std::vector<byte> result(readLength + 1);
        int rd = receiveMessage(servoId, readLength + 1, result.data());
        if (rd < 0) return rd;
        for (int i = 0; i < readLength; i++) outputBuffer[i] = result[i + 1];
        return 0;
    }

    int receiveMessage(byte const &servoId, byte const &readLength, byte *outputBuffer) {
        if (this->dirPin_ < 255) {
            digitalWrite(dirPin_, LOW);
        }
        std::vector<byte> result(readLength + 5);
        size_t rd = port_->readBytes(result.data(), result.size());
        if (rd != static_cast<size_t>(readLength + 5)) return -1;
        if (result[0] != 0xFF || result[1] != 0xFF || result[2] != servoId || result[3] != readLength + 1) return -2;
        byte checksum = 0;
        for (int i = 2; i < readLength + 4; i++) checksum += result[i];
        checksum = (byte)~checksum;
        if (result[readLength + 4] != checksum) return -3;
        for (int i = 0; i < readLength; i++) outputBuffer[i] = result[i + 4];
        return 0;
    }

    void sendAndUpdateChecksum(byte convertedValue[], byte &checksum) {
        port_->write(convertedValue, 2);
        checksum += convertedValue[0] + convertedValue[1];
    }

    void convertIntToBytes(byte const &servoId, int const &value, byte result[2]) {
        uint16_t servoValue = 0;
        if (servoType_[servoId] == ServoType::UNKNOWN) {
            determineServoType(servoId);
        }
        switch (servoType_[servoId]) {
            case ServoType::SCS: {
                int v = std::abs(value);
                servoValue = static_cast<uint16_t>(v);
                if (value < 0) servoValue = 0x0400 | servoValue;
                servoValue = static_cast<uint16_t>((servoValue >> 8) + ((servoValue & 0xFF) << 8));
                break;
            }
            case ServoType::STS:
            default: {
                int v = std::abs(value);
                servoValue = static_cast<uint16_t>(v);
                if (value < 0) servoValue = 0x8000 | servoValue;
                break;
            }
        }
        result[0] = static_cast<unsigned char>(servoValue & 0xFF);
        result[1] = static_cast<unsigned char>((servoValue >> 8) & 0xFF);
    }

    void determineServoType(byte const &servoId) {
        switch (readRegister(servoId, STSRegisters::SERVO_MAJOR)) {
            case 9: servoType_[servoId] = ServoType::STS; break;
            case 5: servoType_[servoId] = ServoType::SCS; break;
            default: break;
        }
    }

private:
    HardwareSerial *port_;
    byte dirPin_;
    ServoType servoType_[256];
};

#endif // STSSERVO_DRIVER_HPP
