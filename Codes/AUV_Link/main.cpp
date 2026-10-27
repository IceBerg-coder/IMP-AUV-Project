// STM32 Nucleo-F411RE side of the Raspberry Pi 5 link. Protocol: PROTOCOL.md
//
// Failsafe: if no valid frame arrives from the Pi for 3 s, outputs go to
// thrust 0, rudder 0, ballast -100 (surface) until a new SETPOINT arrives.
//
// Actuator outputs are not wired yet; apply_outputs() is the place to add
// the ESC PWM, rudder servo and ballast motor driver code.
#include "mbed.h"
#include "link_protocol.h"

using namespace std::chrono_literals;
using namespace link;

static constexpr auto FAILSAFE_TIMEOUT = 3s;
static constexpr auto TELEM_PERIOD     = 200ms;
static constexpr auto LEAK_PERIOD      = 200ms;

static BufferedSerial pi_link(D1, D0, 115200);  // TX = D1 (PA_9), RX = D0 (PA_10)
DigitalOut led(LED1);

// Leak sensor (same wiring as Codes/Leak Sensor)
static AnalogIn  leak_analog(A0);   // AO: 1.0 dry, falls toward 0.0 when wet
static DigitalIn leak_digital(D2);  // DO: 0 when the comparator trips (water)
static bool      leak_latched = false;
static uint8_t   leak_wetness = 0;
static uint8_t   leak_alert = 0;

struct Outputs {
    int8_t thrust;   // -100..100 %
    int8_t rudder;   // -45..45 deg offset from centre
    int8_t ballast;  // -100 = empty (surface), +100 = fill (sink)
};

static Outputs outputs = {0, 0, -100};
static bool failsafe = true;   // stays set until the first valid SETPOINT
static Timer rx_timer;

static int8_t clamp_i8(int v, int lo, int hi)
{
    if (v < lo) return (int8_t)lo;
    if (v > hi) return (int8_t)hi;
    return (int8_t)v;
}

static void apply_outputs(const Outputs &o)
{
    // TODO: thrust -> ESC PWM, rudder -> servo angle (90 + rudder),
    //       ballast -> DRV8833 channels (see Codes/DRV8833)
    (void)o;
}

static void send_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t buf[4 + MAX_PAYLOAD + 1];
    size_t n = encode(type, payload, len, buf, sizeof buf);
    if (n > 0) pi_link.write(buf, n);
}

static void enter_failsafe()
{
    outputs = {0, 0, -100};
    apply_outputs(outputs);
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

static void send_telemetry()
{
    bool link_alive = rx_timer.elapsed_time() <= FAILSAFE_TIMEOUT;
    uint8_t status = (failsafe ? 0x01 : 0x00) | (link_alive ? 0x02 : 0x00)
                   | (leak_latched ? 0x04 : 0x00);
    uint8_t payload[4] = {
        status,
        (uint8_t)outputs.thrust,
        (uint8_t)outputs.rudder,
        (uint8_t)outputs.ballast,
    };
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
        if (p.len() == 3) {
            const uint8_t *d = p.payload();
            outputs.thrust  = clamp_i8((int8_t)d[0], -100, 100);
            outputs.rudder  = clamp_i8((int8_t)d[1], -45, 45);
            outputs.ballast = clamp_i8((int8_t)d[2], -100, 100);
            failsafe = false;
            apply_outputs(outputs);
        }
        break;

    default:
        break;
    }
}

int main()
{
    apply_outputs(outputs);  // start safe: ballast emptied
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

        if (telem_timer.elapsed_time() > TELEM_PERIOD) {
            send_telemetry();
            telem_timer.reset();
        }

        ThisThread::sleep_for(1ms);
    }
}
