#include "mbed.h"
#include <chrono>
#include <cmath>

using namespace std::chrono;

// ---- Console: ST-Link virtual COM port at 115200 baud ----
static BufferedSerial pc(USBTX, USBRX, 115200);
namespace mbed {
FileHandle *mbed_override_console(int)
{
    return &pc;
}
}

constexpr float PI_F    = 3.14159265358979f;
constexpr float RAD2DEG = 180.0f / PI_F;

// Nucleo-F411RE: SDA = D14 (PB_9), SCL = D15 (PB_8)
I2C i2c(PB_9, PB_8);

constexpr int  MPU_ADDR   = 0x68 << 1;   // 8-bit address (AD0 -> GND)
constexpr char REG_WHO    = 0x75;
constexpr char REG_PWR    = 0x6B;
constexpr char REG_SMPLRT = 0x19;
constexpr char REG_CONFIG = 0x1A;
constexpr char REG_GYRO   = 0x1B;
constexpr char REG_ACCEL  = 0x1C;
constexpr char REG_DATA   = 0x3B;

// ---- Float -> text without printf float support ----
// 2 decimals, e.g. -0.98
void fmt2(char *out, size_t n, float v)
{
    int s = (int)lroundf(v * 100.0f);
    const char *sign = (s < 0) ? "-" : "";
    if (s < 0) s = -s;
    snprintf(out, n, "%s%d.%02d", sign, s / 100, s % 100);
}

// 1 decimal, e.g. 12.3
void fmt1(char *out, size_t n, float v)
{
    int s = (int)lroundf(v * 10.0f);
    const char *sign = (s < 0) ? "-" : "";
    if (s < 0) s = -s;
    snprintf(out, n, "%s%d.%d", sign, s / 10, s % 10);
}

bool writeReg(char reg, char val)
{
    char cmd[2] = {reg, val};
    return i2c.write(MPU_ADDR, cmd, 2) == 0;
}

bool readRegs(char reg, char *buf, int len)
{
    if (i2c.write(MPU_ADDR, &reg, 1, true) != 0) return false;  // repeated start
    return i2c.read(MPU_ADDR, buf, len) == 0;
}

void i2cScan()
{
    printf("Scanning I2C bus...\r\n");
    int found = 0;
    for (int addr = 1; addr < 127; addr++) {
        char dummy;
        if (i2c.read(addr << 1, &dummy, 1) == 0) {
            printf("  Device found at 0x%02X\r\n", addr);
            found++;
        }
    }
    if (!found) printf("  No devices found. Check wiring / pull-ups.\r\n");
}

int main()
{
    ThisThread::sleep_for(3s);   // time to click Connect in the dashboard
    printf("\r\n=== MPU-6050 test ===\r\n");
    i2c.frequency(100000);       // start slow (100 kHz) for easier debugging

    i2cScan();

    char id = 0;
    if (!readRegs(REG_WHO, &id, 1)) {
        printf("ERROR: cannot read WHO_AM_I. Check SDA/SCL wiring and AD0.\r\n");
        while (true) ThisThread::sleep_for(1s);
    }
    printf("WHO_AM_I = 0x%02X ", id);
    if (id == 0x68)                    printf("(genuine MPU-6050)\r\n");
    else if (id == 0x70 || id == 0x72) printf("(clone/compatible, continuing)\r\n");
    else                               printf("(unexpected, continuing anyway)\r\n");

    writeReg(REG_PWR, 0x00);      // wake up
    ThisThread::sleep_for(100ms);
    writeReg(REG_SMPLRT, 0x07);   // 1 kHz sample rate
    writeReg(REG_CONFIG, 0x03);   // low-pass filter ~44 Hz
    writeReg(REG_ACCEL, 0x00);    // ±2g
    writeReg(REG_GYRO, 0x00);     // ±250 °/s

    i2c.frequency(400000);        // speed up after init
    printf("Init OK. Reading data...\r\n\r\n");

    float roll = 0, pitch = 0;
    Timer t;
    t.start();

    while (true) {
        char buf[14];
        if (!readRegs(REG_DATA, buf, 14)) {
            printf("read error\r\n");
            ThisThread::sleep_for(200ms);
            continue;
        }

        int16_t ax = (int16_t)((uint8_t)buf[0]  << 8 | (uint8_t)buf[1]);
        int16_t ay = (int16_t)((uint8_t)buf[2]  << 8 | (uint8_t)buf[3]);
        int16_t az = (int16_t)((uint8_t)buf[4]  << 8 | (uint8_t)buf[5]);
        int16_t tr = (int16_t)((uint8_t)buf[6]  << 8 | (uint8_t)buf[7]);
        int16_t gx = (int16_t)((uint8_t)buf[8]  << 8 | (uint8_t)buf[9]);
        int16_t gy = (int16_t)((uint8_t)buf[10] << 8 | (uint8_t)buf[11]);
        int16_t gz = (int16_t)((uint8_t)buf[12] << 8 | (uint8_t)buf[13]);

        float Ax = ax / 16384.0f, Ay = ay / 16384.0f, Az = az / 16384.0f;
        float Gx = gx / 131.0f,   Gy = gy / 131.0f,   Gz = gz / 131.0f;
        float tempC = tr / 340.0f + 36.53f;

        float dt = duration<float>{t.elapsed_time()}.count();
        t.reset();

        float accRoll  = atan2f(Ay, Az) * RAD2DEG;
        float accPitch = atan2f(-Ax, sqrtf(Ay * Ay + Az * Az)) * RAD2DEG;
        roll  = 0.98f * (roll  + Gx * dt) + 0.02f * accRoll;
        pitch = 0.98f * (pitch + Gy * dt) + 0.02f * accPitch;

        char sAx[16], sAy[16], sAz[16], sGx[16], sGy[16], sGz[16], sT[16], sR[16], sP[16];
        fmt2(sAx, sizeof sAx, Ax);  fmt2(sAy, sizeof sAy, Ay);  fmt2(sAz, sizeof sAz, Az);
        fmt1(sGx, sizeof sGx, Gx);  fmt1(sGy, sizeof sGy, Gy);  fmt1(sGz, sizeof sGz, Gz);
        fmt1(sT,  sizeof sT,  tempC);
        fmt1(sR,  sizeof sR,  roll); fmt1(sP, sizeof sP, pitch);

        printf("A[g] %s %s %s | G[dps] %s %s %s | T %sC | Roll %s Pitch %s\r\n",
               sAx, sAy, sAz, sGx, sGy, sGz, sT, sR, sP);

        ThisThread::sleep_for(100ms);
    }
}