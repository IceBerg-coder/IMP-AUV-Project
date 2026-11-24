# IMP AUV Project

Prototype autonomous underwater vehicle (AUV) built around an STM32 Nucleo board running Mbed OS. This repository holds the firmware, PC-side test tools, CAD models, test results and project documents.

## Repository layout

| Folder | Contents |
|---|---|
| `Codes/AUV_Link` | **Main STM32 firmware**: Pi link protocol with failsafe, PCA9685 (servos and ESC), MPU-6050 attitude, MS5837 depth, leak latch and N20 ballast. See `PROTOCOL.md`; host tests in `test/` |
| `Codes/Pi` | Raspberry Pi 5 software: link client `auv_link.py` (with tests), camera test and YOLO object detection test |
| `Codes/ESC_&_Thruster` | Bench test for the ZTW 40A ESC and X2 thruster through the PCA9685 (serial-controlled throttle) |
| `Codes/DRV8833` | N20 motor control through a DRV8833 driver (`main.cpp`, with a 3 s command failsafe) and a Python GUI (`drv8833.py`) |
| `Codes/Leak Sensor` | Water/leak sensor readout (`main.cpp`) with a Python GUI (`leak_gui.py`) |
| `Codes/MPU6050` | MPU6050 IMU readout (`main.cpp`) with an HTML dashboard |
| `Codes/Servo Motors` | Two-servo control (`main.cpp`) with a Python GUI (`servo_gui.py`) |
| `CAD model` | FreeCAD models: AUV prototype and ballast drive |
| `Testing Results` | Photos, videos and data from component, camera, detection, thruster and hull tests |
| `Project Docs` | Project outline, risk assessment, logbooks and power diagram |

The component test folders also have a prebuilt `.bin` that can be flashed to the board. `Codes/AUV_Link` is built in Mbed Studio from its `.cpp`/`.h` files.

## Hardware

- Raspberry Pi 5 with Camera Module v2 (IMX219)
- STM32 Nucleo-F411RE (ST-Link virtual COM port at 115200 baud; Pi link on USART1 PA_9/PA_10)
- PCA9685 PWM driver for two MG90S rudder servos and the ESC
- ZTW Shark G2 40A ESC with an APISQUEEN X2 thruster
- DRV8833 dual motor driver with two N20 ballast motors
- MPU-6050 IMU and MS5837-30BA depth sensor (I2C)
- Rain/leak sensor with LM393 comparator
- 3S LiPo battery with buck converters for the 5 V and 6 V rails

## Usage

1. Build the `main.cpp` of the component you want with Mbed OS, or flash the `.bin` in its folder.
2. Connect the board over USB and note its COM port.
3. Run the matching Python GUI (needs `pyserial`):

```bash
pip install pyserial
python "Codes/DRV8833/drv8833.py"
```

### Serial protocols

- **DRV8833:** `SPEED:<-1.0..1.0>`, `BRAKE`, `STOP`
- **Leak sensor:** the board sends `RAIN:<wetness %>,<alert 0|1>` every 200 ms

### Tests

```bash
# STM32 link protocol (on a PC, from Codes/AUV_Link)
g++ -std=c++17 -Wall -Wextra -I. -o test_link test/test_link_protocol.cpp link_protocol.cpp && ./test_link
# Pi link client (from Codes/Pi)
python3 -m unittest test_auv_link -v
```

## Status

Work in progress. Components are tested individually, and full integration into the vehicle is still to come.
