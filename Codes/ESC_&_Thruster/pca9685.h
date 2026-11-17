// Minimal driver for the PCA9685 16-channel I2C PWM board.
// Used for the two rudder servos and the ESC (all at 50 Hz).
#pragma once

#include "mbed.h"

class PCA9685 {
public:
    // addr7: 7-bit I2C address (0x40 by default, A0-A5 all open).
    PCA9685(I2C &bus, uint8_t addr7 = 0x40);

    // Set the PWM frequency and start the chip. Returns false if no ACK.
    bool begin(float freq_hz);

    // Set a channel's pulse width in microseconds (channel 0..15).
    bool set_pulse_us(uint8_t channel, uint16_t pulse_us);

private:
    bool write_reg(uint8_t reg, uint8_t val);

    I2C    &bus_;
    int     addr_;     // 8-bit address for Mbed
    float   freq_hz_;
};
