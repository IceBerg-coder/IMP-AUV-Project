#include "link_protocol.h"

namespace link {

uint8_t crc8_update(uint8_t crc, uint8_t byte)
{
    crc ^= byte;
    for (int i = 0; i < 8; i++) {
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

size_t encode(uint8_t type, const uint8_t *payload, uint8_t len,
              uint8_t *out, size_t out_cap)
{
    if (len > MAX_PAYLOAD) return 0;
    size_t total = FRAME_OVERHEAD + len;
    if (out_cap < total) return 0;

    out[0] = SYNC1;
    out[1] = SYNC2;
    out[2] = len;
    out[3] = type;
    uint8_t crc = crc8_update(0, len);
    crc = crc8_update(crc, type);
    for (uint8_t i = 0; i < len; i++) {
        out[4 + i] = payload[i];
        crc = crc8_update(crc, payload[i]);
    }
    out[4 + len] = crc;
    return total;
}

void Parser::reset()
{
    state_ = WAIT_SYNC1;
    len_ = 0;
    type_ = 0;
    idx_ = 0;
    crc_ = 0;
}

bool Parser::feed(uint8_t byte)
{
    switch (state_) {
    case WAIT_SYNC1:
        if (byte == SYNC1) state_ = WAIT_SYNC2;
        return false;

    case WAIT_SYNC2:
        if (byte == SYNC2)       state_ = WAIT_LEN;
        else if (byte == SYNC1)  state_ = WAIT_SYNC2;  // could be the start of a new frame
        else                     state_ = WAIT_SYNC1;
        return false;

    case WAIT_LEN:
        if (byte > MAX_PAYLOAD) {
            reset();
            return false;
        }
        len_ = byte;
        crc_ = crc8_update(0, byte);
        state_ = WAIT_TYPE;
        return false;

    case WAIT_TYPE:
        type_ = byte;
        crc_ = crc8_update(crc_, byte);
        idx_ = 0;
        state_ = (len_ == 0) ? WAIT_CRC : IN_PAYLOAD;
        return false;

    case IN_PAYLOAD:
        payload_[idx_++] = byte;
        crc_ = crc8_update(crc_, byte);
        if (idx_ == len_) state_ = WAIT_CRC;
        return false;

    case WAIT_CRC: {
        bool ok = (byte == crc_);
        state_ = WAIT_SYNC1;
        return ok;
    }
    }
    return false;
}

}  // namespace link
