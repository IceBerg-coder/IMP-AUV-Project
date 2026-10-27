#include "imu_mpu6050.h"

#include <cmath>

using namespace std::chrono_literals;

namespace {
constexpr uint8_t REG_PWR_MGMT_1  = 0x6B;
constexpr uint8_t REG_SMPLRT_DIV  = 0x19;
constexpr uint8_t REG_CONFIG      = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT  = 0x3B;   // 14 bytes: accel, temp, gyro
constexpr uint8_t REG_WHO_AM_I    = 0x75;

constexpr float ACCEL_LSB_PER_G   = 16384.0f;  // +/-2 g
constexpr float GYRO_LSB_PER_DPS  = 131.0f;   // +/-250 deg/s
constexpr float RAD2DEG           = 180.0f / 3.14159265358979f;
constexpr float COMP_GYRO_WEIGHT  = 0.98f;
}  // namespace

MPU6050::MPU6050(I2C &bus, uint8_t addr7)
    : bus_(bus), addr_(addr7 << 1)
{
}

bool MPU6050::write_reg(uint8_t reg, uint8_t val)
{
    char d[2] = {(char)reg, (char)val};
    return bus_.write(addr_, d, 2) == 0;
}

bool MPU6050::read_regs(uint8_t reg, uint8_t *buf, int len)
{
    char r = (char)reg;
    if (bus_.write(addr_, &r, 1, true) != 0) return false;   // repeated start
    return bus_.read(addr_, (char *)buf, len) == 0;
}

bool MPU6050::begin()
{
    uint8_t id = 0;
    if (!read_regs(REG_WHO_AM_I, &id, 1)) return false;
    // 0x68 is a genuine MPU-6050; 0x70/0x72 are compatible clones seen on some boards
    if (id != 0x68 && id != 0x70 && id != 0x72) return false;

    if (!write_reg(REG_PWR_MGMT_1, 0x00)) return false;   // wake up
    ThisThread::sleep_for(100ms);
    if (!write_reg(REG_SMPLRT_DIV, 0x07)) return false;   // 1 kHz / (1 + 7) = 125 Hz
    if (!write_reg(REG_CONFIG, 0x03)) return false;       // low-pass filter ~44 Hz
    if (!write_reg(REG_ACCEL_CONFIG, 0x00)) return false; // +/-2 g
    if (!write_reg(REG_GYRO_CONFIG, 0x00)) return false;  // +/-250 deg/s

    healthy_ = true;
    return true;
}

bool MPU6050::read_gyro_accel(float &ax, float &ay, float &az,
                              float &gx, float &gy, float &gz)
{
    uint8_t b[14];
    if (!read_regs(REG_ACCEL_XOUT, b, sizeof b)) return false;

    auto s16 = [](uint8_t hi, uint8_t lo) {
        return (int16_t)(((uint16_t)hi << 8) | lo);
    };
    ax = s16(b[0],  b[1])  / ACCEL_LSB_PER_G;
    ay = s16(b[2],  b[3])  / ACCEL_LSB_PER_G;
    az = s16(b[4],  b[5])  / ACCEL_LSB_PER_G;
    gx = s16(b[8],  b[9])  / GYRO_LSB_PER_DPS;
    gy = s16(b[10], b[11]) / GYRO_LSB_PER_DPS;
    gz = s16(b[12], b[13]) / GYRO_LSB_PER_DPS;
    return true;
}

void MPU6050::calibrate_gyro(uint32_t samples, uint32_t period_ms)
{
    double sx = 0, sy = 0, sz = 0;
    uint32_t n = 0;
    for (uint32_t i = 0; i < samples; i++) {
        float ax, ay, az, gx, gy, gz;
        if (read_gyro_accel(ax, ay, az, gx, gy, gz)) {
            sx += gx; sy += gy; sz += gz;
            n++;
        }
        ThisThread::sleep_for(std::chrono::milliseconds(period_ms));
    }
    if (n > 0) {
        bias_gx_ = (float)(sx / n);
        bias_gy_ = (float)(sy / n);
        bias_gz_ = (float)(sz / n);
    }
}

bool MPU6050::update(float dt_s)
{
    float ax, ay, az, gx, gy, gz;
    if (!read_gyro_accel(ax, ay, az, gx, gy, gz)) {
        healthy_ = false;
        return false;
    }
    healthy_ = true;

    gx -= bias_gx_;
    gy -= bias_gy_;

    float acc_roll  = atan2f(ay, az) * RAD2DEG;
    float acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;

    roll_  = COMP_GYRO_WEIGHT * (roll_  + gx * dt_s) + (1.0f - COMP_GYRO_WEIGHT) * acc_roll;
    pitch_ = COMP_GYRO_WEIGHT * (pitch_ + gy * dt_s) + (1.0f - COMP_GYRO_WEIGHT) * acc_pitch;
    return true;
}
