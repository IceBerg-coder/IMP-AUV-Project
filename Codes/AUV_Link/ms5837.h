// MS5837-30BA pressure and temperature sensor driver (Bar30 variant).
// Shares the I2C bus with the MPU-6050 and PCA9685. Depth is measured in fresh water.
#pragma once

#include "mbed.h"

class MS5837 {
public:
    // addr7: 0x76 with CSB low (default), 0x77 with CSB high.
    explicit MS5837(I2C &bus, uint8_t addr7 = 0x76);

    // Reset and read the factory calibration. False if no ACK or bad calibration.
    bool begin();

    // Read pressure (mbar) and temperature (deg C). False on an I2C error.
    bool read(float &pressure_mbar, float &temp_c);

    // Average readings with the vehicle out of the water to set the surface reference.
    void zero_surface(uint32_t samples, uint32_t period_ms);

    // Depth in metres below the zeroed surface (fresh water). Valid after zero_surface().
    float depth_m() const { return depth_m_; }

    // Call read() then update depth in one go. False on an I2C error.
    bool update_depth();

    bool healthy() const { return healthy_; }

private:
    bool write_cmd(uint8_t cmd);
    bool read_adc(uint32_t &value);
    bool read_prom(uint16_t &word, uint8_t index);
    bool convert(uint8_t cmd, uint32_t &value);

    I2C     &bus_;
    int      addr_;     // 8-bit address for Mbed
    bool     healthy_ = false;
    uint16_t c_[7] = {0};    // C1..C6 used (index 1..6)
    float    surface_mbar_ = 1013.25f;
    float    depth_m_ = 0.0f;
};
