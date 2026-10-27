# Raspberry Pi 5 ↔ STM32 Nucleo-F411RE link protocol

Serial link: UART, 115200 baud, 8N1, no flow control.

- Pi 5 TX (GPIO14) → Nucleo **PA_10** (USART1 RX)
- Pi 5 RX (GPIO15) ← Nucleo **PA_9** (USART1 TX)
- GND shared between the Pi and the Nucleo.

Do not use D0 or D1 for this link. On the Nucleo-F411RE they are USART2, which is the ST-Link USB console. Find PA_9 and PA_10 in the Nucleo-64 user manual (UM1724) pin table, which lists the Arduino header pins.

## Frame format

| Byte | Field | Notes |
|---|---|---|
| 0 | SYNC1 | `0xAA` |
| 1 | SYNC2 | `0x55` |
| 2 | LEN | payload length, 0–16 |
| 3 | TYPE | message type (below) |
| 4 … 3+LEN | PAYLOAD | LEN bytes |
| 4+LEN | CRC8 | CRC-8, polynomial `0x07`, init `0x00`, computed over LEN, TYPE and PAYLOAD (not the sync bytes) |

Multi-byte integers are little-endian. Signed values are two's complement.

## Messages

| TYPE | Name | Direction | Payload |
|---|---|---|---|
| `0x01` | HEARTBEAT | Pi → MCU | `seq` (u8). Send every 500 ms. |
| `0x02` | SETPOINT | Pi → MCU | `thrust` (i8, −100…100 %), `yaw` (i8, −45…45° rudder offset), `pitch` (i8, −45…45° rudder offset), `ballast` (i8, −100…100 %) |
| `0x03` | ACK | MCU → Pi | `seq` (u8), echoed from the HEARTBEAT |
| `0x10` | TELEMETRY | MCU → Pi | `status` (u8), `thrust` (i8), `yaw` (i8), `pitch` (i8), `ballast` (i8), `attitude_roll` (i16, tenths of a degree), `attitude_pitch` (i16, tenths of a degree). Sent every 200 ms. |
| `0x11` | LEAK | MCU → Pi | `wetness` (u8, 0–100 %), `alert` (u8, 1 = water detected). Sent every 200 ms. |

`status` bits: bit 0 = failsafe active, bit 1 = link alive, bit 2 = leak latched, bit 3 = IMU fault (last IMU read failed).

## Leak sensor

- The leak sensor is read on the STM32: AO → A0 (wetness), DO → D4 (alert, active low from the comparator).
- `wetness = (1 − A0) × 100`, clamped to 0–100, the same formula as `Codes/Leak Sensor`.
- When `alert` is 1, the STM32 **latches the leak**: thrust 0, rudders 0, ballast −100 (surface). SETPOINT frames are ignored while latched.
- The latch clears only on power-up or reset, so a leak can't be cleared by a SETPOINT from the Pi.
- The Pi should stop mission control and abort the run when it receives a leak alert.

## Attitude

- The STM32 reads the MPU-6050 and runs a complementary filter (98 % gyro, 2 % accelerometer).
- At boot, the gyro is calibrated while the vehicle is held still for about 2 s.
- `attitude_roll` and `attitude_pitch` are in tenths of a degree, so 123 means 12.3°.
- The IMU does not drive the failsafe. Its state is only reported, through the status bit.

## Sign conventions

- **thrust:** +100 = full forward, −100 = full reverse.
- **yaw / pitch:** 0 = centred. Servo angle = 90 + offset. Yaw is the left/right rudder pair, pitch is the up/down pair (X-cross layout, two servos).
- **ballast:** +100 = fill syringes (sink), −100 = empty syringes (surface).

## Failsafe

- Any valid frame from the Pi resets a 3 s timer.
- At boot, the ESC is held at neutral (1.5 ms) for 3 s before thrust is accepted.
- If no valid frame arrives for 3 s, the MCU enters **failsafe**: thrust 0, rudders 0, ballast −100 (surface). The ballast motors stop after 10 s of emptying so they aren't left stalled at the syringe end-stops (placeholder, calibrate). It stays there until a valid SETPOINT arrives.
- A HEARTBEAT alone keeps the link alive but does not clear failsafe. The Pi must send a SETPOINT to resume control.
- The Pi should send a SETPOINT (even a zero one) every 500 ms while it's in control.

## Golden test vectors

| Message | Bytes |
|---|---|
| HEARTBEAT seq 7 | `AA 55 01 01 07 6B` |
| SETPOINT thrust 50, yaw −10, pitch 0, ballast 100 | `AA 55 04 02 32 F6 00 64 CF` |
| ACK seq 7 | `AA 55 01 03 07 41` |
| TELEMETRY status 1, thrust 50, yaw −10, pitch 0, ballast 100, roll 0, pitch 0 | `AA 55 09 10 01 32 F6 00 64 00 00 00 00 21` |
| LEAK wetness 85, alert 1 | `AA 55 02 11 55 01 AF` |

Both test suites check these bytes.
