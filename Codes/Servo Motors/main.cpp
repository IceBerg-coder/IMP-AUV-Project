#include "mbed.h"
#include <cstdio>

PwmOut servo1(D5);
PwmOut servo2(D6);
BufferedSerial pc(USBTX, USBRX, 115200);

static int us_for_angle(int deg)
{
    if (deg < 0)   deg = 0;
    if (deg > 180) deg = 180;
    return 1000 + (deg * 1000) / 180;
}

static void set_servo(int id, int deg)
{
    if (id == 1)      servo1.pulsewidth_us(us_for_angle(deg));
    else if (id == 2) servo2.pulsewidth_us(us_for_angle(deg));
}

int main()
{
    servo1.period(0.02f);
    servo2.period(0.02f);
    servo1.pulsewidth_us(1500);
    servo2.pulsewidth_us(1500);

    char line[32];
    size_t len = 0;

    while (true) {
        char c;
        if (pc.read(&c, 1) != 1) continue;

        if (c == '\n') {
            line[len] = '\0';
            int id, deg;
            if (sscanf(line, "%d %d", &id, &deg) == 2) {
                set_servo(id, deg);
            }
            len = 0;
        } else if (c != '\r' && len < sizeof(line) - 1) {
            line[len++] = c;
        }
    }
}