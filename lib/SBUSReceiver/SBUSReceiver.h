#pragma once
#include <Arduino.h>

class SBUSReceiver {
public:
    SBUSReceiver(HardwareSerial& serial, int rxPin, int txPin);
    void begin();
    bool readFrame();
    uint16_t getChannel(uint8_t ch) const;
    bool getCh17() const;
    bool getCh18() const;
    bool isFailsafe() const;
    bool isFrameLost() const;
    bool isLostConnection() const;

private:
    void decode(const uint8_t *data);

    HardwareSerial& serial_;
    int rxPin_, txPin_;
    static constexpr uint32_t SBUS_BAUD = 100000;

    uint8_t sbusData_[25];
    uint16_t channels_[16];
    bool ch17_, ch18_, failsafe_, frameLost_;
};
