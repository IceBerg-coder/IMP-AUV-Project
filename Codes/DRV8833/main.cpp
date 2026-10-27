#include "mbed.h"
#include <chrono>
#include <cstring>
#include <cstdlib>

using namespace std::chrono_literals;

// Hardware Pin setup using Arduino header aliases D5 and D6
// D5 -> PB_4 (TIM3_CH1)
// D6 -> PB_10 (TIM2_CH3)
PwmOut motor_in1(D5);
PwmOut motor_in2(D6);
DigitalOut led(LED1);

// USB Serial interface (ST-Link Virtual COM Port)
static BufferedSerial pc(USBTX, USBRX, 115200);

// Failsafe: if no valid command arrives within this time, stop the motor.
static constexpr auto CMD_TIMEOUT = 3s;
static Timer cmd_timer;
static bool motor_stopped_by_timeout = false;

void motor_set_speed(float speed) {
    if (speed > 1.0f)  speed = 1.0f;
    if (speed < -1.0f) speed = -1.0f;

    if (speed > 0.0f) {
        // Forward
        motor_in1.write(speed);
        motor_in2.write(0.0f);
        led = 1;
    } else if (speed < 0.0f) {
        // Reverse
        motor_in1.write(0.0f);
        motor_in2.write(-speed);
        led = 1;
    } else {
        // Coast / Stop
        motor_in1.write(0.0f);
        motor_in2.write(0.0f);
        led = 0;
    }
}

void motor_brake() {
    motor_in1.write(1.0f);
    motor_in2.write(1.0f);
    led = 0;
}

int main() {
    // Set 20 kHz PWM frequency (period = 50 us)
    motor_in1.period_us(50);
    motor_in2.period_us(50);

    // Ensure motor starts stopped
    motor_set_speed(0.0f);

    cmd_timer.start();

    char rx_buf[64];
    int buf_idx = 0;

    while (true) {
        char ch;
        if (pc.read(&ch, 1) > 0) {
            if (ch == '\n' || ch == '\r') {
                if (buf_idx > 0) {
                    rx_buf[buf_idx] = '\0';

                    // Parse commands from Python GUI
                    bool valid = true;
                    if (strncmp(rx_buf, "SPEED:", 6) == 0) {
                        float speed = atof(&rx_buf[6]);
                        motor_set_speed(speed);
                    } else if (strcmp(rx_buf, "BRAKE") == 0) {
                        motor_brake();
                    } else if (strcmp(rx_buf, "STOP") == 0) {
                        motor_set_speed(0.0f);
                    } else {
                        valid = false;
                    }

                    if (valid) {
                        cmd_timer.reset();
                        motor_stopped_by_timeout = false;
                    }
                    buf_idx = 0;
                }
            } else if (buf_idx < (int)sizeof(rx_buf) - 1) {
                rx_buf[buf_idx++] = ch;
            }
        }

        // Failsafe check runs every pass, even when no bytes arrive
        if (!motor_stopped_by_timeout && cmd_timer.elapsed_time() > CMD_TIMEOUT) {
            motor_set_speed(0.0f);
            motor_stopped_by_timeout = true;
        }
    }
}