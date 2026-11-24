// MPU-6050 IMU driver with a complementary filter for roll and pitch.
// Shares the I2C bus with the PCA9685 and the MS5837 (D14 SDA, D15 SCL).
#pragma once

#include "mbed.h"

class MPU6050 {
public:
    // addr7: 0x68 when AD0 is low (default).
    explicit MPU6050(I2C &bus, uint8_t addr7 = 0x68);

    // Wake the chip and set the ranges (+/-2 g, +/-250 deg/s). False if no ACK or wrong ID.
    bool begin();

    // Average the gyro while the vehicle is held still, to remove the bias.
    // Call once after begin(), before the vehicle moves.
    void calibrate_gyro(uint32_t samples, uint32_t period_ms);

    // Read the sensor and update roll and pitch. dt_s is the time since the last call.
    // Returns false on an I2C error; the last good angles are kept.
    bool update(float dt_s);

    bool  healthy() const { return healthy_; }
    float roll_deg() const { return roll_; }
    float pitch_deg() const { return pitch_; }

private:
    bool read_gyro_accel(float &ax, float &ay, float &az,
                         float &gx, float &gy, float &gz);
    bool write_reg(uint8_t reg, uint8_t val);
    bool read_regs(uint8_t reg, uint8_t *buf, int len);

    I2C   &bus_;
    int    addr_;      // 8-bit address for Mbed
    bool   healthy_ = false;
    float  roll_ = 0.0f;
    float  pitch_ = 0.0f;
    float  bias_gx_ = 0.0f;
    float  bias_gy_ = 0.0f;
    float  bias_gz_ = 0.0f;
};
