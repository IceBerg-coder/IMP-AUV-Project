# Raspberry Pi 5 ↔ STM32 Nucleo-F411RE link protocol

Serial link: UART, 115200 baud, 8N1, no flow control.

- Pi 5 TX (GPIO14) → Nucleo **D0** (PA_10, USART1 RX)
- Pi 5 RX (GPIO15) ← Nucleo **D1** (PA_9, USART1 TX)
- GND shared between the Pi and the Nucleo.

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
| `0x02` | SETPOINT | Pi → MCU | `thrust` (i8, −100…100 %), `rudder` (i8, −45…45° offset from centre), `ballast` (i8, −100…100 %) |
| `0x03` | ACK | MCU → Pi | `seq` (u8), echoed from the HEARTBEAT |
| `0x10` | TELEMETRY | MCU → Pi | `status` (u8), `thrust` (i8), `rudder` (i8), `ballast` (i8). Sent every 200 ms. |
| `0x11` | LEAK | MCU → Pi | `wetness` (u8, 0–100 %), `alert` (u8, 1 = water detected). Sent every 200 ms. |

`status` bits: bit 0 = failsafe active, bit 1 = link alive, bit 2 = leak latched.

## Leak sensor

- The leak sensor is read on the STM32: AO → A0 (wetness), DO → D2 (alert, active low from the comparator).
- `wetness = (1 − A0) × 100`, clamped to 0–100, the same formula as `Codes/Leak Sensor`.
- When `alert` is 1, the STM32 **latches the leak**: thrust 0, rudder 0, ballast −100 (surface). SETPOINT frames are ignored while latched.
- The latch clears only on power-up or reset, so a leak can't be cleared by a SETPOINT from the Pi.
- The Pi should stop mission control and abort the run when it receives a leak alert.

## Sign conventions

- **thrust:** +100 = full forward, −100 = full reverse.
- **rudder:** 0 = centred. Servo angle = 90 + rudder.
- **ballast:** +100 = fill syringes (sink), −100 = empty syringes (surface).

## Failsafe

- Any valid frame from the Pi resets a 3 s timer.
- If no valid frame arrives for 3 s, the MCU enters **failsafe**: thrust 0, rudder 0, ballast −100 (surface). It stays there until a valid SETPOINT arrives.
- A HEARTBEAT alone keeps the link alive but does not clear failsafe. The Pi must send a SETPOINT to resume control.
- The Pi should send a SETPOINT (even a zero one) every 500 ms while it's in control.

## Golden test vectors

| Message | Bytes |
|---|---|
| HEARTBEAT seq 7 | `AA 55 01 01 07 6B` |
| SETPOINT thrust 50, rudder −10, ballast 100 | `AA 55 03 02 32 F6 64 EC` |
| ACK seq 7 | `AA 55 01 03 07 41` |
| TELEMETRY status 1, thrust 50, rudder −10, ballast 100 | `AA 55 04 10 01 32 F6 64 E6` |
| LEAK wetness 85, alert 1 | `AA 55 02 11 55 01 AF` |

Both test suites check these bytes.
