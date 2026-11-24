// Bench test: ZTW Shark G2 40A ESC + APISQUEEN X2 thruster through the PCA9685.
//
// Hardware
//   Nucleo-F411RE (USB to PC for power and the serial console, 115200 baud)
//   PCA9685: VCC <- Nucleo 3.3 V, GND, SDA <- D14 (PB_9), SCL <- D15 (PB_8)
//   ESC signal (white/yellow) + ground (black/brown) -> PCA9685 channel 2
//   ESC red BEC wire: leave disconnected
//   ESC power: current-limited bench supply (12 V, 3 A limit) or the 3S LiPo
//
// Project files: this main.cpp + pca9685.h + pca9685.cpp (from Codes/AUV_Link)
//
// Keys (in the serial terminal)
//   w  throttle +5 %        s  throttle -5 % (negative = reverse)
//   space or 0  stop (neutral)
//   p  print status
// Throttle is capped at +/-30 % for the bench and returns to neutral on its own
// if no key is pressed for 10 s.

#include "mbed.h"
#include "pca9685.h"

#include <algorithm>

using namespace std::chrono_literals;

static BufferedSerial console(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int) { return &console; }

static I2C       i2c(PB_9, PB_8);
static PCA9685   pwm(i2c, 0x40);
static DigitalOut led(LED1);

static constexpr uint8_t CH_ESC      = 2;
static constexpr int     NEUTRAL_US  = 1500;   // stop in forward/reverse mode
static constexpr int     US_PER_PCT  = 5;      // 100 % = +/-500 us (1.0 to 2.0 ms)
static constexpr int     MAX_PCT     = 30;     // bench cap; raise only in water, clamped
static constexpr int     STEP_PCT    = 5;
static constexpr auto    ARM_TIME    = 5s;
static constexpr auto    DEADMAN     = 10s;

static int throttle_pct = 0;

static void apply()
{
    int us = NEUTRAL_US + throttle_pct * US_PER_PCT;
    bool ok = pwm.set_pulse_us(CH_ESC, (uint16_t)us);
    printf("throttle %+4d %%  pulse %4d us  %s\n", throttle_pct, us, ok ? "" : "(I2C write failed)");
}

int main()
{
    printf("\n=== ESC / X2 bench test ===\n");

    if (!pwm.begin(50.0f)) {
        printf("PCA9685 not found at 0x40. Check VCC (3.3 V), GND, SDA D14, SCL D15.\n");
        while (true) {
            led = !led;
            ThisThread::sleep_for(100ms);   // fast blink = PCA9685 missing
        }
    }

    throttle_pct = 0;
    apply();
    printf("Sending neutral (1500 us). Power the ESC now and wait for its arming beeps.\n");

    Timer arm;
    arm.start();
    while (arm.elapsed_time() < ARM_TIME) {
        led = !led;
        ThisThread::sleep_for(250ms);
    }
    led = 1;
    printf("Ready. w = +5 %%, s = -5 %%, space/0 = stop, p = status. Cap +/-%d %%.\n", MAX_PCT);

    Timer idle;
    idle.start();

    while (true) {
        if (console.readable()) {
            char c;
            if (console.read(&c, 1) == 1) {
                idle.reset();
                switch (c) {
                case 'w': case 'W':
                    throttle_pct = std::min(throttle_pct + STEP_PCT, MAX_PCT);
                    apply();
                    break;
                case 's': case 'S':
                    throttle_pct = std::max(throttle_pct - STEP_PCT, -MAX_PCT);
                    apply();
                    break;
                case ' ': case '0':
                    throttle_pct = 0;
                    apply();
                    break;
                case 'p': case 'P':
                    apply();
                    break;
                default:
                    break;
                }
            }
        }

        if (throttle_pct != 0 && idle.elapsed_time() > DEADMAN) {
            throttle_pct = 0;
            printf("No key for 10 s: back to neutral.\n");
            apply();
        }

        ThisThread::sleep_for(10ms);
    }
}
