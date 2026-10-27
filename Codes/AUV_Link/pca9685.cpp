#include "pca9685.h"

using namespace std::chrono_literals;

namespace {
constexpr uint8_t REG_MODE1    = 0x00;
constexpr uint8_t REG_PRESCALE = 0xFE;
constexpr uint8_t REG_LED0     = 0x06;   // channel n: LED0 + 4*n
constexpr float   OSC_HZ       = 25000000.0f;

constexpr uint8_t MODE1_SLEEP   = 0x10;
constexpr uint8_t MODE1_AI      = 0x20;  // auto-increment
constexpr uint8_t MODE1_RESTART = 0x80;
}  // namespace

PCA9685::PCA9685(I2C &bus, uint8_t addr7)
    : bus_(bus), addr_(addr7 << 1), freq_hz_(50.0f)
{
}

bool PCA9685::write_reg(uint8_t reg, uint8_t val)
{
    char d[2] = {(char)reg, (char)val};
    return bus_.write(addr_, d, 2) == 0;
}

bool PCA9685::begin(float freq_hz)
{
    freq_hz_ = freq_hz;
    uint8_t prescale = (uint8_t)(OSC_HZ / (4096.0f * freq_hz) + 0.5f) - 1;

    if (!write_reg(REG_MODE1, MODE1_SLEEP | MODE1_AI)) return false;  // sleep before changing prescale
    if (!write_reg(REG_PRESCALE, prescale)) return false;
    if (!write_reg(REG_MODE1, MODE1_AI)) return false;                // wake
    ThisThread::sleep_for(5ms);                                       // oscillator settle time
    return write_reg(REG_MODE1, MODE1_RESTART | MODE1_AI);
}

bool PCA9685::set_pulse_us(uint8_t channel, uint16_t pulse_us)
{
    if (channel > 15) return false;

    uint32_t period_us = (uint32_t)(1000000.0f / freq_hz_ + 0.5f);
    uint32_t ticks = ((uint32_t)pulse_us * 4096u) / period_us;
    if (ticks > 4095u) ticks = 4095u;

    // ON = 0, OFF = ticks (12-bit each, little-endian byte pairs)
    char d[5] = {
        (char)(REG_LED0 + 4 * channel),
        0, 0,
        (char)(ticks & 0xFF),
        (char)((ticks >> 8) & 0x0F),
    };
    return bus_.write(addr_, d, 5) == 0;
}
