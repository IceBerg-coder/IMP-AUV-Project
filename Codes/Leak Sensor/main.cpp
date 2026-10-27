#include "mbed.h"
#include <chrono>

using namespace std::chrono_literals;

// Rain Sensor Pins
AnalogIn rain_analog(A0);      // AO: 0.0 (wet) to 1.0 (dry)
DigitalIn rain_digital(D2);    // DO: 0 when rain triggers comparator, 1 when dry

// Onboard user LED for visual rain alert
DigitalOut led(LED1);

// ST-Link Virtual COM Port
static BufferedSerial pc(USBTX, USBRX, 115200);

int main() {
    char tx_buf[64];

    while (true) {
        // Read raw analog level (0.0 to 1.0) and convert to moisture percentage
        // Dry plate is ~1.0, saturated plate drops toward 0.0
        float wetness = (1.0f - rain_analog.read()) * 100.0f;
        if (wetness < 0.0f)   wetness = 0.0f;
        if (wetness > 100.0f) wetness = 100.0f;

        // Digital alert: active LOW from LM393 comparator
        int rain_alert = (rain_digital.read() == 0) ? 1 : 0;

        // Turn on the board LED if rain is detected
        led = rain_alert;

        // Send telemetry line: "RAIN:<wetness_percentage>,<alert_flag>\n"
        int len = snprintf(tx_buf, sizeof(tx_buf), "RAIN:%.1f,%d\r\n", wetness, rain_alert);
        pc.write(tx_buf, len);

        rtos::ThisThread::sleep_for(200ms);
    }
}