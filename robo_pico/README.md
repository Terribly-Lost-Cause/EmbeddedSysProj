# LandShark Robot Controller

This project implements a dual-core Raspberry Pi Pico robot controller system. It features a primary robot controller (`robo_pico`) that manages motors, sensors (IMU, Encoders), and odometry, and a secondary sender unit (`maker_pi_pico`) that issues autonomous command sequences via UART.

## Features

*   **Precise Navigation:** Uses IMU (Inertial Measurement Unit) and Wheel Encoders for accurate turning and distance measurement.
*   **Odometry Tracking:** Real-time calculation of robot pose (X, Y, Theta) fusing encoder data with IMU heading.
*   **Drift Correction:** Active heading correction while moving forward to maintain a straight path.
*   **UART Communication:** Robust command and telemetry protocol between the controller and sender units.
*   **Autonomous Sequences:** Pre-programmed movement sequences (e.g., square path, figure-8) executed by the sender.
*   **Telemetry Feedback:** Real-time reporting of position and status ("done", "none") to the sender.

## Hardware Requirements

*   2x Raspberry Pi Pico (one for Robot, one for Sender/Remote)
*   Motor Driver (compatible with PWM control)
*   DC Motors with Encoders
*   IMU Sensor (e.g., BNO055 or similar I2C IMU)
*   Chassis and Power Supply (Battery)
*   UART Connection between Picos (TX/RX crossed)

## Software Requirements

*   [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)
*   CMake (3.13 or later)
*   GCC ARM Embedded Toolchain

## Project Structure

```
.
├── CMakeLists.txt          # Build configuration
├── pico_sdk_import.cmake   # Pico SDK import helper
├── include/                # Header files
│   ├── drift_correction.h
│   ├── encoder.h
│   ├── imu.h
│   ├── motors.h
│   └── uart_comm.h
└── src/                    # Source files
    ├── robo_pico.c         # MAIN TARGET: Robot Controller firmware
    ├── maker_pi_pico.c     # MAIN TARGET: Command Sender firmware
    ├── motors.c            # Motor control driver
    ├── imu.c               # IMU sensor driver
    ├── encoder.c           # Encoder interrupt handling
    ├── drift_correction.c  # Heading correction logic
    ├── uart_comm.c         # UART communication helper
    └── ...                 # Various test files (auto_turn_test, etc.)
```

## Build Instructions

1.  **Clone the repository:**
    ```bash
    git clone <repository_url>
    cd <repository_name>
    ```

2.  **Create a build directory:**
    ```bash
    mkdir build
    cd build
    ```

3.  **Configure CMake:**
    ```bash
    cmake ..
    ```

4.  **Build the project:**
    ```bash
    make
    ```
    *This will generate `.uf2` files for all targets in the `build` directory.*

## Usage

### 1. Robot Controller (`robo_pico`)
Flash `robo_pico.uf2` to the robot's Raspberry Pi Pico.
*   **Startup:** The robot performs a 2-second forward auto-test.
*   **Operation:** Waits for UART commands.
    *   `'w'`: Move Forward (with drift correction)
    *   `'s'`: Move Backward
    *   `'a'`: Turn Left (90 degrees)
    *   `'d'`: Turn Right (90 degrees)
    *   `'q'`: Stop
*   **Telemetry:** Sends `PX,x,y,theta,status` messages back to the sender.

### 2. Command Sender (`maker_pi_pico`)
Flash `maker_pi_pico.uf2` to the second Raspberry Pi Pico.
*   **Operation:** Automatically sends a sequence of commands to the robot.
*   **Sequences:**
    1.  4x Right Turns (Square)
    2.  4x Left Turns (Square)
    3.  Right -> Left -> Right -> Left (Zig-Zag)
*   **Synchronization:** Waits for a "done" message from the robot before sending the next command.

## Communication Protocol

*   **Baud Rate:** Default (usually 115200 or 9600, check `uart_comm.c`)
*   **Command Format:** Single character (`w`, `a`, `s`, `d`, `q`).
*   **Telemetry Format:** `PX,<x_pos>,<y_pos>,<theta_deg>,<status>`
    *   Example: `PX,1.2,0.5,90.0,done`
