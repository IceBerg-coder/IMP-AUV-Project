# IMP AUV Project

Prototype autonomous underwater vehicle (AUV) built around an STM32 Nucleo board running Mbed OS. This repository holds the firmware, PC-side test tools, CAD models, test results and project documents.

## Repository layout

| Folder | Contents |
|---|---|
| `Codes/DRV8833` | N20 motor control through a DRV8833 driver (`main.cpp`) with a Python GUI (`drv8833.py`) |
| `Codes/Leak Sensor` | Water/leak sensor readout (`main.cpp`) with a Python GUI (`leak_gui.py`) |
| `Codes/MPU6050` | MPU6050 IMU readout (`main.cpp`) with an HTML dashboard |
| `Codes/Servo Motors` | Two-servo control (`main.cpp`) with a Python GUI (`servo_gui.py`) |
| `CAD model` | FreeCAD models: AUV prototype and ballast drive |
| `Testing Results` | Photos and videos from component tests |
| `Project Docs` | Project outline, risk assessment, logbooks and power diagram |

Each firmware folder also has a prebuilt `.bin` that can be flashed to the board.

## Hardware

- STM32 Nucleo board (ST-Link virtual COM port at 115200 baud)
- DRV8833 dual motor driver with N20 motors
- Rain/leak sensor with LM393 comparator
- MPU6050 IMU
- Hobby servo motors
- 3S LiPo battery

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

## Status

Work in progress. Components are tested individually, and full integration into the vehicle is still to come.
