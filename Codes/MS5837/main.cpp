// MS5837-30BA bench and bucket test for the Nucleo-F411RE (Mbed OS 6).
//
// Wiring (GY-MS5837 module):  VCC -> 3V3   GND -> GND   SDA -> D14 (PB_9)   SCL -> D15 (PB_8)
// The bare sensor is rated 1.5-3.6 V, so use 3V3, not 5 V, unless the module has its own regulator.
//
// Serial console: ST-Link virtual COM port at 115200 baud.
//   z : re-zero the surface pressure (do this with the sensor in air)
//   o : step the oversampling rate (OSR 256 ... 8192)
//
// Output, one line every 200 ms (CSV, easy to paste into Excel):
//   t_ms,pressure_mbar,temp_c,depth_cm,noise_mm
// noise_mm is the standard deviation of depth over the last 25 readings (5 s).

#include "mbed.h"
#include "ms5837.h"
#include <cmath>

using namespace std::chrono;

static BufferedSerial pc(USBTX, USBRX, 115200);
namespace mbed {
FileHandle *mbed_override_console(int) { return &pc; }
}

I2C i2c(PB_9, PB_8);
MS5837 sensor(i2c, 0x76);

static const int OSR_VALUE[6] = {256, 512, 1024, 2048, 4096, 8192};

// Float to text with a fixed number of decimals (minimal printf has no %f)
static void fmt(char *out, size_t n, float v, int dp)
{
    long scale = 1;
    for (int i = 0; i < dp; i++) scale *= 10;
    long s = lroundf(v * scale);
    const char *sign = (s < 0) ? "-" : "";
    if (s < 0) s = -s;
    if (dp == 0) snprintf(out, n, "%s%ld", sign, s);
    else snprintf(out, n, "%s%ld.%0*ld", sign, s / scale, dp, s % scale);
}

static void scan_bus()
{
    printf("I2C scan:");
    int found = 0;
    for (int a = 0x08; a < 0x78; a++) {
        char d;
        if (i2c.read(a << 1, &d, 1) == 0) {   // 0 means the address was acknowledged
            printf(" 0x%02X", a);
            found++;
        }
    }
    printf(found ? "\r\n" : " nothing found - check wiring and power\r\n");
}

static void zero_now()
{
    printf("# zeroing surface pressure, keep the sensor in air and still...\r\n");
    sensor.zero_surface(25, 40);
    printf("# zero done\r\n");
}

int main()
{
    ThisThread::sleep_for(500ms);
    printf("\r\n# MS5837-30BA test\r\n");
    scan_bus();

    while (!sensor.begin()) {
        printf("# MS5837 not found at 0x76 or PROM CRC failed, retrying in 2 s\r\n");
        ThisThread::sleep_for(2s);
    }
    printf("# sensor OK, version 0x%02X (%s)\r\n", sensor.version(),
           sensor.version() == 0x1A ? "30BA" : sensor.version() == 0x15 ? "02BA - wrong model?" : "unknown");

    uint8_t osr = 5;            // start with the lowest-noise setting
    sensor.set_osr(osr);
    zero_now();

    const int N = 25;
    float hist[N] = {0};
    int count = 0, head = 0;
    Timer t;
    t.start();
    printf("t_ms,pressure_mbar,temp_c,depth_cm,noise_mm\r\n");

    while (true) {
        // Keyboard commands
        while (pc.readable()) {
            char c;
            if (pc.read(&c, 1) == 1) {
                if (c == 'z' || c == 'Z') { zero_now(); count = 0; head = 0; }
                if (c == 'o' || c == 'O') {
                    osr = (osr + 1) % 6;
                    sensor.set_osr(osr);
                    count = 0; head = 0;
                    printf("# OSR %d\r\n", OSR_VALUE[osr]);
                }
            }
        }

        float p, temp;
        if (!sensor.read(p, temp)) {
            printf("# read error\r\n");
            ThisThread::sleep_for(500ms);
            continue;
        }
        sensor.update_depth();
        float d_m = sensor.depth_m();

        hist[head] = d_m;
        head = (head + 1) % N;
        if (count < N) count++;
        float mean = 0, var = 0;
        for (int i = 0; i < count; i++) mean += hist[i];
        mean /= count;
        for (int i = 0; i < count; i++) var += (hist[i] - mean) * (hist[i] - mean);
        float noise_mm = (count > 1) ? sqrtf(var / (count - 1)) * 1000.0f : 0.0f;

        char sp[16], st[16], sd[16], sn[16];
        fmt(sp, sizeof sp, p, 2);
        fmt(st, sizeof st, temp, 2);
        fmt(sd, sizeof sd, d_m * 100.0f, 1);
        fmt(sn, sizeof sn, noise_mm, 1);
        printf("%lu,%s,%s,%s,%s\r\n",
               (unsigned long)duration_cast<milliseconds>(t.elapsed_time()).count(), sp, st, sd, sn);

        ThisThread::sleep_for(200ms);
    }
}
