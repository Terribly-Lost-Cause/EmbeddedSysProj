# LandShark Robot Project

This repository contains the complete firmware for the **LandShark**, a dual-microcontroller autonomous robot. The system is split into two distinct modules: the **Sensor Module** (Brain) and the **Motor Controller** (Muscles), each running on its own Raspberry Pi Pico.

## 📂 Project Structure

The project is organized into two main subdirectories, each functioning as an independent CMake project:

```
.
├── robo_pico/              # Motor Controller Firmware (The "Muscles")
│   ├── CMakeLists.txt      # Build configuration for Motor Controller
│   ├── include/            # Shared headers (motors.h, imu.h, etc.)
│   └── src/
│       ├── robo_pico.c     # [MAIN] UART-controlled driver. Receives commands from Sensor Pico.
│       ├── main.c          # [DEMO] Standalone PID control demo (drives straight with IMU).
│       ├── maker_pi_pico.c # [TEST] Simple UART sender to test robo_pico without sensors.
│       ├── motors.c        # PWM motor control logic.
│       ├── imu.c           # BNO055 IMU driver.
│       ├── encoder.c       # Quadrature encoder interrupt handler.
│       └── ...
│
└── sensor_pico/            # Sensor Module Firmware (The "Brain")
    ├── CMakeLists.txt      # Build configuration for Sensor Module
    ├── include/            # Shared headers (lidar.h, mqtt.h, etc.)
    └── src/
        ├── sensor_pico.c   # [MAIN] Core FSM. Handles Navigation, Lidar, and Decision Making.
        ├── lidar.c         # Driver for RPLidar (or similar) via UART.
        ├── servo_mmwave_pan.c # Driver for Servo + mmWave Radar scanning.
        ├── mqtt.c          # WiFi telemetry and mapping data publishing.
        └── uart_comm.c     # Custom PIO-UART for communicating with robo_pico.
```

## 🧠 System Architecture

The two Picos communicate via a custom UART protocol.

1.  **Sensor Module (`sensor_pico`)**:
    *   **Role:** High-level decision making, obstacle avoidance, and mapping.
    *   **Hardware:** Lidar, mmWave Radar, Servo, WiFi.
    *   **Logic:** Runs a State Machine (Forward -> Detect -> Decide -> Turn). Sends single-character commands (`W`, `A`, `S`, `D`, `Q`) to the Motor Controller.

2.  **Motor Controller (`robo_pico`)**:
    *   **Role:** Low-level hardware abstraction and execution.
    *   **Hardware:** DC Motors, Encoders, IMU.
    *   **Logic:** Executes movement commands while maintaining heading (Drift Correction). Reports telemetry (`PX,x,y,theta,status`) back to the Sensor Module.

## 🚀 Getting Started

### Prerequisites
*   **Hardware:** 2x Raspberry Pi Pico (Pico W for Sensor Module), Motor Driver, Motors, IMU, Lidar, mmWave Radar.
*   **Software:** [Pico SDK](https://github.com/raspberrypi/pico-sdk), CMake, ARM GCC Toolchain.

### Building the Projects

You must build each module separately.

#### 1. Build Motor Controller (`robo_pico`)
This firmware goes onto the Pico connected to the motors.
```bash
cd robo_pico
mkdir build
cd build
cmake ..
make robo_pico
```
*Output:* `robo_pico/build/robo_pico.uf2`

#### 2. Build Sensor Module (`sensor_pico`)
This firmware goes onto the Pico connected to the Lidar/Radar.
```bash
cd sensor_pico
mkdir build
cd build
cmake ..
make sensor_pico
```
*Output:* `sensor_pico/build/sensor_pico.uf2`

## 🔌 Wiring & Communication

Connect the two Picos via UART:
*   **Sensor Pico TX** -> **Motor Pico RX**
*   **Sensor Pico RX** -> **Motor Pico TX**
*   **GND** -> **GND** (Common Ground is critical!)

### Protocol
*   **Control (Sensor -> Motor):**
    *   `W`: Forward (with drift correction)
    *   `S`: Backward
    *   `A`: Turn Left 90°
    *   `D`: Turn Right 90°
    *   `Q`: Stop
*   **Telemetry (Motor -> Sensor):**
    *   Format: `PX,<x>,<y>,<theta>,<status>`
    *   Example: `PX,100.0,50.0,90.0,done`

## 🧪 Testing Tools

The `robo_pico` folder includes additional targets for testing:
*   **`main` (`main.c`)**: A standalone test that drives the robot straight using cascaded PID control. Useful for tuning motor/IMU parameters without the full system.
*   **`maker_pi_pico` (`maker_pi_pico.c`)**: A "dummy sender" that mimics the Sensor Pico. It sends a fixed sequence of turns to test the Motor Controller's response.
