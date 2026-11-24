#include "ms5837.h"

#include <cmath>

using namespace std::chrono_literals;

namespace {
constexpr uint8_t CMD_RESET     = 0x1E;
constexpr uint8_t CMD_ADC_READ  = 0x00;
constexpr uint8_t CMD_PROM_BASE = 0xA0;   // PROM word i at 0xA0 + 2*i
constexpr uint8_t CMD_D1_OSR256 = 0x40;   // pressure, OSR 256 (~0.6 ms conversion)
constexpr uint8_t CMD_D2_OSR256 = 0x50;   // temperature, OSR 256

constexpr float FRESH_WATER_DENSITY = 1000.0f;   // kg/m^3
constexpr float GRAVITY = 9.80665f;               // m/s^2
}  // namespace

MS5837::MS5837(I2C &bus, uint8_t addr7)
    : bus_(bus), addr_(addr7 << 1)
{
}

bool MS5837::write_cmd(uint8_t cmd)
{
    char c = (char)cmd;
    return bus_.write(addr_, &c, 1) == 0;
}

bool MS5837::read_adc(uint32_t &value)
{
    if (!write_cmd(CMD_ADC_READ)) return false;
    uint8_t b[3];
    if (bus_.read(addr_, (char *)b, 3) != 0) return false;
    value = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
    return true;
}

bool MS5837::read_prom(uint16_t &word, uint8_t index)
{
    if (!write_cmd(CMD_PROM_BASE + 2 * index)) return false;
    uint8_t b[2];
    if (bus_.read(addr_, (char *)b, 2) != 0) return false;
    word = ((uint16_t)b[0] << 8) | b[1];
    return true;
}

bool MS5837::begin()
{
    if (!write_cmd(CMD_RESET)) return false;
    ThisThread::sleep_for(10ms);

    for (uint8_t i = 1; i <= 6; i++) {
        if (!read_prom(c_[i], i)) return false;
    }
    // A calibration word of all zeros means the sensor or its PROM is not readable
    for (uint8_t i = 1; i <= 6; i++) {
        if (c_[i] == 0) return false;
    }
    healthy_ = true;
    return true;
}

bool MS5837::convert(uint8_t cmd, uint32_t &value)
{
    if (!write_cmd(cmd)) return false;
    ThisThread::sleep_for(2ms);   // OSR 256 conversion is under 1 ms; 2 ms leaves margin
    return read_adc(value);
}

bool MS5837::read(float &pressure_mbar, float &temp_c)
{
    uint32_t d1 = 0, d2 = 0;
    if (!convert(CMD_D2_OSR256, d2)) { healthy_ = false; return false; }
    if (!convert(CMD_D1_OSR256, d1)) { healthy_ = false; return false; }

    // First-order compensation (datasheet)
    int32_t dT   = (int32_t)d2 - (int32_t)c_[5] * 256;
    int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * c_[6]) >> 23);

    int64_t OFF  = (int64_t)c_[2] * 65536 + ((int64_t)c_[4] * dT) / 128;
    int64_t SENS = (int64_t)c_[1] * 32768 + ((int64_t)c_[3] * dT) / 256;

    // Second-order compensation for the MS5837-30BA (below 20 C)
    int64_t Ti = 0, OFFi = 0, SENSi = 0;
    if (TEMP < 2000) {
        Ti = (3LL * dT * dT) >> 33;
        int64_t t = TEMP - 2000;
        OFFi = (3 * t * t) / 2;
        SENSi = (5 * t * t) / 8;
        if (TEMP < -1500) {
            int64_t t2 = TEMP + 1500;
            OFFi  += 7 * t2 * t2;
            SENSi += (4 * t2 * t2);
        }
    } else {
        Ti = (2LL * dT * dT) >> 37;
    }

    TEMP -= (int32_t)Ti;
    OFF  -= OFFi;
    SENS -= SENSi;

    int32_t P = (int32_t)((((int64_t)d1 * SENS) >> 21) - OFF) >> 13;

    temp_c = TEMP / 100.0f;
    pressure_mbar = P / 10.0f;   // the datasheet gives tenths of a mbar
    healthy_ = true;
    return true;
}

void MS5837::zero_surface(uint32_t samples, uint32_t period_ms)
{
    double sum = 0.0;
    uint32_t n = 0;
    for (uint32_t i = 0; i < samples; i++) {
        float p, t;
        if (read(p, t)) {
            sum += p;
            n++;
        }
        ThisThread::sleep_for(std::chrono::milliseconds(period_ms));
    }
    if (n > 0) surface_mbar_ = (float)(sum / n);
}

bool MS5837::update_depth()
{
    float p, t;
    if (!read(p, t)) return false;
    // 1 mbar = 100 Pa. Depth = gauge pressure / (density * g)
    float gauge_pa = (p - surface_mbar_) * 100.0f;
    depth_m_ = gauge_pa / (FRESH_WATER_DENSITY * GRAVITY);
    return true;
}
