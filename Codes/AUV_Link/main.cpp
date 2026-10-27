// STM32 Nucleo-F411RE side of the Raspberry Pi 5 link. Protocol: PROTOCOL.md
//
// Outputs
//   Rudder yaw servo   PCA9685 channel 0   (50 Hz)
//   Rudder pitch servo PCA9685 channel 1   (50 Hz)
//   45A ESC            PCA9685 channel 2   (50 Hz, 1.0-2.0 ms, neutral 1.5 ms)
//   N20 ballast A      D3 (IN1), D6 (IN2)  (20 kHz, DRV8833 channel A)
//   N20 ballast B      D5 (IN1), D7 (IN2)  (20 kHz, DRV8833 channel B)
//
// Safety
//   - Boot: all outputs neutral; ESC held at neutral for ARM_TIME.
//   - No valid frame from the Pi for FAILSAFE_TIMEOUT: thrust 0, rudders 0,
//     ballast -100 (empty = surface). Ballast drive stops after EMPTY_LIMIT
//     so the N20s are not left stalled at the syringe end-stops.
//   - Leak alert: latched failsafe until power cycle; SETPOINTs ignored.
#include "mbed.h"
#include <cmath>
#include "link_protocol.h"
#include "pca9685.h"
#include "imu_mpu6050.h"
#include "ms5837.h"

using namespace std::chrono_literals;
using namespace link;

static constexpr auto FAILSAFE_TIMEOUT = 3s;
static constexpr auto TELEM_PERIOD     = 200ms;
static constexpr auto LEAK_PERIOD      = 200ms;
static constexpr auto ARM_TIME         = 5s;
static constexpr auto EMPTY_LIMIT      = 10s;   // placeholder: calibrate to syringe stroke

static constexpr uint8_t CH_YAW   = 0;
static constexpr uint8_t CH_PITCH = 1;
static constexpr uint8_t CH_ESC   = 2;

// Pi link on USART1 (PA_9 = TX, PA_10 = RX). Not D0/D1 (those are the ST-Link console).
static BufferedSerial pi_link(PA_9, PA_10, 115200);
DigitalOut led(LED1);

// Leak sensor: AO on A0, DO on D4 (D2 is the Pi link RX on this board)
static AnalogIn  leak_analog(A0);
static DigitalIn leak_digital(D4);
static bool      leak_latched = false;
static uint8_t   leak_wetness = 0;
static uint8_t   leak_alert = 0;

// PCA9685 on the shared I2C bus (D14 = PB_9 SDA, D15 = PB_8 SCL)
static I2C    i2c(PB_9, PB_8);
static PCA9685 pwm(i2c, 0x40);
static MPU6050 imu(i2c, 0x68);
static Timer   imu_timer;
static MS5837  depth_sensor(i2c, 0x76);
static Timer   depth_timer;

// N20 ballast motors on the DRV8833 (20 kHz)
static PwmOut bal_a_in1(D3);
static PwmOut bal_a_in2(D6);
static PwmOut bal_b_in1(D5);
static PwmOut bal_b_in2(D7);

struct Outputs {
    int8_t thrust;   // -100..100 %
    int8_t yaw;      // -45..45 deg offset from centre
    int8_t pitch;    // -45..45 deg offset from centre
    int8_t ballast;  // -100 = empty (surface), +100 = fill (sink)
};

static Outputs outputs = {0, 0, 0, -100};
static bool    failsafe = true;   // stays set until the first valid SETPOINT
static Timer   rx_timer;
static Timer   arm_timer;
static Timer   empty_timer;       // time since failsafe ballast-empty started

static int8_t clamp_i8(int v, int lo, int hi)
{
    if (v < lo) return (int8_t)lo;
    if (v > hi) return (int8_t)hi;
    return (int8_t)v;
}

static void drive_motor(PwmOut &in1, PwmOut &in2, float s)
{
    if (s > 0.0f) {
        in1.write(s);   in2.write(0.0f);
    } else if (s < 0.0f) {
        in1.write(0.0f); in2.write(-s);
    } else {
        in1.write(0.0f); in2.write(0.0f);
    }
}

// Angle 0..180 deg -> pulse width 1000..2000 us (matches Codes/Servo Motors)
static uint16_t servo_us(int angle_deg)
{
    if (angle_deg < 0) angle_deg = 0;
    if (angle_deg > 180) angle_deg = 180;
    return (uint16_t)(1000 + (angle_deg * 1000) / 180);
}

// Called every loop pass. Idempotent: writes the current outputs to the hardware.
static void apply_outputs()
{
    // ESC: thrust -100..100 maps to 1000..2000 us; held at neutral until armed
    bool armed = arm_timer.elapsed_time() > ARM_TIME;
    int esc_us = armed ? 1500 + outputs.thrust * 5 : 1500;
    pwm.set_pulse_us(CH_ESC, (uint16_t)esc_us);

    // Rudders: 90 deg is centre
    pwm.set_pulse_us(CH_YAW,   servo_us(90 + outputs.yaw));
    pwm.set_pulse_us(CH_PITCH, servo_us(90 + outputs.pitch));

    // Ballast: both syringes move together. In failsafe, stop after EMPTY_LIMIT.
    float b = outputs.ballast / 100.0f;
    if (failsafe && outputs.ballast <= -100 && empty_timer.elapsed_time() > EMPTY_LIMIT) {
        b = 0.0f;
    }
    drive_motor(bal_a_in1, bal_a_in2, b);
    drive_motor(bal_b_in1, bal_b_in2, b);
}

static void send_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t buf[FRAME_OVERHEAD + MAX_PAYLOAD];
    size_t n = encode(type, payload, len, buf, sizeof buf);
    if (n > 0) pi_link.write(buf, n);
}

static void enter_failsafe()
{
    outputs = {0, 0, 0, -100};
    if (!failsafe) empty_timer.reset();
    failsafe = true;
}

static void read_leak()
{
    float wet = (1.0f - leak_analog.read()) * 100.0f;
    if (wet < 0.0f)   wet = 0.0f;
    if (wet > 100.0f) wet = 100.0f;
    leak_wetness = (uint8_t)wet;
    leak_alert = (leak_digital.read() == 0) ? 1 : 0;

    if (leak_alert && !leak_latched) {
        leak_latched = true;      // stays latched until power cycle
        enter_failsafe();
    }
}

static void send_leak()
{
    uint8_t payload[2] = {leak_wetness, leak_alert};
    send_frame(MSG_LEAK, payload, sizeof payload);
}

static void put_i16(uint8_t *dst, int v)
{
    dst[0] = (uint8_t)(v & 0xFF);
    dst[1] = (uint8_t)((v >> 8) & 0xFF);
}

static void send_telemetry()
{
    bool link_alive = rx_timer.elapsed_time() <= FAILSAFE_TIMEOUT;
    uint8_t status = (failsafe ? 0x01 : 0x00) | (link_alive ? 0x02 : 0x00)
                   | (leak_latched ? 0x04 : 0x00) | (imu.healthy() ? 0x00 : 0x08)
                   | (depth_sensor.healthy() ? 0x00 : 0x10);

    // Attitude in tenths of a degree, int16 little-endian
    int roll10  = (int)lroundf(imu.roll_deg() * 10.0f);
    int pitch10 = (int)lroundf(imu.pitch_deg() * 10.0f);

    // Depth in centimetres, int16 little-endian
    int depth_cm = (int)lroundf(depth_sensor.depth_m() * 100.0f);

    uint8_t payload[11];
    payload[0] = status;
    payload[1] = (uint8_t)outputs.thrust;
    payload[2] = (uint8_t)outputs.yaw;
    payload[3] = (uint8_t)outputs.pitch;
    payload[4] = (uint8_t)outputs.ballast;
    put_i16(&payload[5], roll10);
    put_i16(&payload[7], pitch10);
    put_i16(&payload[9], depth_cm);
    send_frame(MSG_TELEMETRY, payload, sizeof payload);
}

static void handle_frame(const Parser &p)
{
    rx_timer.reset();  // any valid frame keeps the link alive

    switch (p.type()) {
    case MSG_HEARTBEAT:
        if (p.len() == 1) {
            uint8_t seq = p.payload()[0];
            send_frame(MSG_ACK, &seq, 1);
        }
        break;

    case MSG_SETPOINT:
        if (leak_latched) break;  // ignore commands while a leak is latched
        if (p.len() == 4) {
            const uint8_t *d = p.payload();
            outputs.thrust  = clamp_i8((int8_t)d[0], -100, 100);
            outputs.yaw     = clamp_i8((int8_t)d[1], -45, 45);
            outputs.pitch   = clamp_i8((int8_t)d[2], -45, 45);
            outputs.ballast = clamp_i8((int8_t)d[3], -100, 100);
            failsafe = false;
        }
        break;

    default:
        break;
    }
}

int main()
{
    // Start safe: ballast emptied, ESC neutral, servos centred
    outputs = {0, 0, 0, -100};
    arm_timer.start();
    empty_timer.start();

    bool pwm_ok = pwm.begin(50.0f);
    (void)pwm_ok;  // TODO: if false, keep outputs off and report via telemetry

    // IMU: start, then calibrate the gyro while the vehicle is held still (~2 s).
    // The ESC arming window (3 s) covers this, so nothing moves during calibration.
    if (imu.begin()) {
        imu.calibrate_gyro(200, 10);
    }
    imu_timer.start();

    // Depth sensor: set the surface reference with the vehicle OUT of the water.
    // Power up in air; the 20 samples take about 1 s.
    if (depth_sensor.begin()) {
        depth_sensor.zero_surface(20, 50);
    }
    depth_timer.start();

    bal_a_in1.period_us(50);
    bal_a_in2.period_us(50);
    bal_b_in1.period_us(50);
    bal_b_in2.period_us(50);

    apply_outputs();

    rx_timer.start();
    Timer telem_timer;
    telem_timer.start();
    Timer leak_timer;
    leak_timer.start();

    Parser parser;

    while (true) {
        while (pi_link.readable()) {
            uint8_t ch;
            if (pi_link.read(&ch, 1) == 1 && parser.feed(ch)) {
                handle_frame(parser);
                led = !led;
            }
        }

        if (leak_timer.elapsed_time() > LEAK_PERIOD) {
            read_leak();
            send_leak();
            leak_timer.reset();
        }

        if (!failsafe && rx_timer.elapsed_time() > FAILSAFE_TIMEOUT) {
            enter_failsafe();
        }

        float dt = std::chrono::duration<float>(imu_timer.elapsed_time()).count();
        imu_timer.reset();
        imu.update(dt);   // a failed read keeps the last angles and sets the IMU fault bit

        if (depth_timer.elapsed_time() > 100ms) {
            depth_sensor.update_depth();   // 10 Hz is enough for the depth loop
            depth_timer.reset();
        }

        apply_outputs();

        if (telem_timer.elapsed_time() > TELEM_PERIOD) {
            send_telemetry();
            telem_timer.reset();
        }

        ThisThread::sleep_for(10ms);
    }
}
