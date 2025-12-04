# Sensor Pico - LandShark Robot Module

This project contains the firmware for the **Sensor Module** of the LandShark robot, running on a Raspberry Pi Pico W. It is responsible for obstacle detection (Lidar & mmWave), sensor fusion, and high-level decision making, communicating with a separate Motor Controller Pico (`robo_pico`).

## Project Structure

*   **Project Name:** `sensor_pico`
*   **Build System:** CMake
*   **Platform:** Raspberry Pi Pico W (using `pico-sdk`)

### Key Files

*   `src/sensor_pico.c`: **Main Application Entry Point.**
    *   **Core 0:** Runs the main Finite State Machine (FSM) for robot behavior, handles UART communication with the motor controller, and manages the mmWave sensor/servo.
    *   **Core 1:** Dedicated to the Lidar driver, continuously scanning and updating obstacle detection flags.
*   `src/uart_comm.c`: Custom UART implementation using **PIO (Programmable I/O)** to communicate with the Motor Controller. Handles parsing of Pose (`PX`) and Status messages.
*   `src/servo_mmwave_pan.c`: Driver for the LD2410 mmWave radar and the panning servo. It sweeps the sensor and detects obstacles in the robot's path.
*   `src/lidar.c`: Driver for the Lidar sensor (e.g., RPLidar or similar), handling serial data parsing.
*   `src/mqtt.c`: Handles Wi-Fi connectivity and MQTT publishing for telemetry and mapping data.

## System Architecture

The system utilizes both cores of the RP2040 to maximize performance:

### Core 0: Brain & Coordination
*   **Initialization:** Sets up all hardware (UARTs, PIO, Servo, WiFi).
*   **Communication:**
    *   Receives **Pose** ($x, y, \theta$) and **Status** ("done", "none") from the Motor Pico.
    *   Sends **Motion Commands** ('W', 'A', 'S', 'D', 'Q') to the Motor Pico.
*   **State Machine:**
    1.  **Forward (State 1):** Robot moves forward ('W') while panning the mmWave sensor.
    2.  **Obstacle Detected:** If Lidar (Front) or mmWave detects an object, it stops ('Q') and transitions to Decision (State 2).
    3.  **Decision (State 2):** Analyzes Lidar data to find a clear path (Left, Right, or Reverse).
    4.  **Action (States 3, 4, 5):** Executes the chosen turn/reverse maneuver and waits for a "done" signal from the Motor Pico before resuming.

### Core 1: Lidar Perception
*   Continuously reads data from the Lidar via Hardware UART.
*   Filters points based on distance and quality.
*   Updates shared **Detection Flags** (`frontDet`, `leftDet`, `rightDet`, `backDet`) used by Core 0.
*   (Optional) Publishes point cloud data over MQTT for remote visualization.

## Hardware Connections

| Component | Pico Pin | Description |
| :--- | :--- | :--- |
| **Lidar** | UART1 (Default) | Hardware UART for high-speed Lidar data. |
| **mmWave** | UART0 (Default) | Hardware UART for radar target data. |
| **Servo** | PWM Pin | Controls the panning mechanism. |
| **Comms (RX)** | GP6 (PIO) | Custom PIO UART RX from Motor Pico. |
| **Comms (TX)** | GP7 (PIO) | Custom PIO UART TX to Motor Pico. |

## Building the Project

1.  Ensure the **Pico SDK** is installed and configured in your environment.
2.  Create a build directory:
    ```bash
    mkdir build
    cd build
    ```
3.  Run CMake:
    ```bash
    cmake ..
    ```
4.  Build the target:
    ```bash
    make sensor_pico
    ```
    *(Or use the "Build" button in VS Code if using the Pico extension)*

## Communication Protocol

The Sensor Pico communicates with the Motor Pico using a text-based protocol:

*   **TX (To Motor Pico):** Single characters for control.
    *   `W`: Move Forward
    *   `S`: Move Backward
    *   `A`: Turn Left
    *   `D`: Turn Right
    *   `Q`: Stop
*   **RX (From Motor Pico):** CSV-formatted strings.
    *   Format: `PX,<x>,<y>,<theta>,<status>`
    *   Example: `PX,100.5,200.0,90.0,done`
    *   `status`: Can be "done" (command finished), "none" (busy/idle), or other info.
