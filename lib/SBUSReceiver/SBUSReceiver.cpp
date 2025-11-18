#include "SBUSReceiver.h"

SBUSReceiver::SBUSReceiver(HardwareSerial& serial, int rxPin, int txPin)
    : serial_(serial), rxPin_(rxPin), txPin_(txPin),
      ch17_(false), ch18_(false), failsafe_(false), frameLost_(false) {
    memset(channels_, 0, sizeof(channels_));
}

void SBUSReceiver::begin() {
    serial_.begin(SBUS_BAUD, SERIAL_8E2, rxPin_, txPin_, true);
}

bool SBUSReceiver::readFrame() {
    static uint8_t idx = 0;

    while (serial_.available()) {
        uint8_t c = serial_.read();

        if (idx == 0 && c != 0x0F) continue;

        sbusData_[idx++] = c;

        if (idx >= 25) {
            idx = 0;
            decode(sbusData_);
            return true;
        }
    }
    return false;
}

void SBUSReceiver::decode(const uint8_t *data) {
    channels_[0]  = ((data[1]      | data[2]  << 8) & 0x07FF);
    channels_[1]  = ((data[2] >> 3 | data[3]  << 5) & 0x07FF);
    channels_[2]  = ((data[3] >> 6 | data[4]  << 2 | data[5] << 10) & 0x07FF);
    channels_[3]  = ((data[5] >> 1 | data[6]  << 7) & 0x07FF);
    channels_[4]  = ((data[6] >> 4 | data[7]  << 4) & 0x07FF);
    channels_[5]  = ((data[7] >> 7 | data[8]  << 1 | data[9] << 9) & 0x07FF);
    channels_[6]  = ((data[9] >> 2 | data[10] << 6) & 0x07FF);
    channels_[7]  = ((data[10] >> 5 | data[11] << 3) & 0x07FF);
    channels_[8]  = ((data[12]     | data[13] << 8) & 0x07FF);
    channels_[9]  = ((data[13] >> 3 | data[14] << 5) & 0x07FF);
    channels_[10] = ((data[14] >> 6 | data[15] << 2 | data[16] << 10) & 0x07FF);
    channels_[11] = ((data[16] >> 1 | data[17] << 7) & 0x07FF);
    channels_[12] = ((data[17] >> 4 | data[18] << 4) & 0x07FF);
    channels_[13] = ((data[18] >> 7 | data[19] << 1 | data[20] << 9) & 0x07FF);
    channels_[14] = ((data[20] >> 2 | data[21] << 6) & 0x07FF);
    channels_[15] = ((data[21] >> 5 | data[22] << 3) & 0x07FF);

    ch17_      = data[23] & 0x80;
    ch18_      = data[23] & 0x40;
    frameLost_ = data[23] & 0x20;
    failsafe_  = data[23] & 0x10;
}

uint16_t SBUSReceiver::getChannel(uint8_t ch) const {
    if (ch < 16) return channels_[ch];
    return 0;
}

bool SBUSReceiver::getCh17() const { return ch17_; }
bool SBUSReceiver::getCh18() const { return ch18_; }
bool SBUSReceiver::isFailsafe() const { return failsafe_; }
bool SBUSReceiver::isFrameLost() const { return frameLost_; }
