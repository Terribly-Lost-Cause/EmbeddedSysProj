cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

# ------------------------------------------------------------------------------
# Project basics
# ------------------------------------------------------------------------------
project(RobotController C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize Pico SDK (must come after project())
pico_sdk_init()

# ------------------------------------------------------------------------------
# Common include path (headers live in ./include)
# ------------------------------------------------------------------------------
set(PROJ_INCLUDE_DIR ${CMAKE_CURRENT_LIST_DIR}/include)

# ==============================================================================
# TARGET 1: robot  — Full integrated app
#   Sources: mainRobot.c + all feature modules (motors, encoder, imu)
#   Use when you want the complete robot (telemetry + drift detection + kill).
# ==============================================================================
add_executable(robot
    src/mainRobot.c
    src/motors.c
    src/encoder.c
    src/imu.c
)

# Headers for this target
target_include_directories(robot PRIVATE ${PROJ_INCLUDE_DIR})

# Link only what this app needs (PWM, I2C for IMU, GPIO for kill/encoders)
target_link_libraries(robot
    pico_stdlib
    hardware_pwm
    hardware_i2c
    hardware_gpio
)

# USB stdio ON (serial monitor), UART stdio OFF
pico_enable_stdio_usb(robot 1)
pico_enable_stdio_uart(robot 0)

# Produce UF2/ELF/BIN for `robot`
pico_add_extra_outputs(robot)

# ==============================================================================
# TARGET 2: motortest  — Minimal manual motor tester (WASD over USB)
#   Sources: motortest.c + motors module only
#   Use for quick bring-up of motor wiring/PWM without IMU/encoders.
# ==============================================================================
add_executable(motortest
    src/motortest.c
    src/motors.c
)

# Headers for this target (same include dir)
target_include_directories(motortest PRIVATE ${PROJ_INCLUDE_DIR})

# Link only what motortest actually uses (PWM + GPIO). No I2C needed here.
target_link_libraries(motortest
    pico_stdlib
    hardware_pwm
    hardware_gpio
)

# USB stdio ON (so you can type commands), UART stdio OFF
pico_enable_stdio_usb(motortest 1)
pico_enable_stdio_uart(motortest 0)

# Produce UF2/ELF/BIN for `motortest`
pico_add_extra_outputs(motortest)

# ------------------------------------------------------------------------------
# Build tips
#   - Both targets compile in one build: you'll get robot.uf2 and motortest.uf2
#   - Flash whichever you need by dragging the corresponding UF2 to RPI-RP2.
# ------------------------------------------------------------------------------
