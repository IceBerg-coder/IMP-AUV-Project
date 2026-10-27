// Framing for the Raspberry Pi 5 <-> STM32 link. See PROTOCOL.md.
#pragma once

#include <cstddef>
#include <cstdint>

namespace link {

constexpr uint8_t SYNC1 = 0xAA;
constexpr uint8_t SYNC2 = 0x55;
constexpr size_t  MAX_PAYLOAD = 16;
constexpr size_t  FRAME_OVERHEAD = 5;  // SYNC1, SYNC2, LEN, TYPE, CRC

enum MsgType : uint8_t {
    MSG_HEARTBEAT = 0x01,
    MSG_SETPOINT  = 0x02,
    MSG_ACK       = 0x03,
    MSG_TELEMETRY = 0x10,
};

// Incremental CRC-8 (poly 0x07, init 0x00).
uint8_t crc8_update(uint8_t crc, uint8_t byte);

// Encode one frame into out. Returns the number of bytes written, or 0 if
// the payload is too long or out_cap is too small.
size_t encode(uint8_t type, const uint8_t *payload, uint8_t len,
              uint8_t *out, size_t out_cap);

// Byte-at-a-time parser. feed() returns true when a complete frame with a
// valid CRC has just been received; the frame is then available through
// type(), len() and payload() until the next call to feed().
class Parser {
public:
    bool feed(uint8_t byte);
    void reset();

    uint8_t type() const { return type_; }
    uint8_t len() const { return len_; }
    const uint8_t *payload() const { return payload_; }

private:
    enum State : uint8_t {
        WAIT_SYNC1, WAIT_SYNC2, WAIT_LEN, WAIT_TYPE, IN_PAYLOAD, WAIT_CRC
    };

    State   state_ = WAIT_SYNC1;
    uint8_t len_ = 0;
    uint8_t type_ = 0;
    uint8_t idx_ = 0;
    uint8_t crc_ = 0;
    uint8_t payload_[MAX_PAYLOAD] = {0};
};

}  // namespace link
