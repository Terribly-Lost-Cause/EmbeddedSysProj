cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

# ------------------------------------------------------------------------------
# Project basics
# ------------------------------------------------------------------------------
project(RobotController C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize Pico SDK
pico_sdk_init()

# ------------------------------------------------------------------------------
# Common include paths
# We look in BOTH 'include' and 'src' to find your .h files
# ------------------------------------------------------------------------------
set(PROJ_INCLUDE_DIRS 
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/src
)

# ==============================================================================
# TARGET 1: robo_pico — Full UART + Time-based turn robo_pico
# ==============================================================================
add_executable(robo_pico
    src/robo_pico.c
    src/motors.c
    src/imu.c
    src/encoder.c
    src/uart_comm.c
)

target_include_directories(robo_pico PRIVATE ${PROJ_INCLUDE_DIRS})

target_link_libraries(robo_pico
    pico_stdlib
    hardware_i2c
    hardware_pwm
    hardware_gpio
    hardware_uart
)

pico_enable_stdio_usb(robo_pico 1)
pico_enable_stdio_uart(robo_pico 0)
pico_add_extra_outputs(robo_pico)

# ==============================================================================
# TARGET 2: maker_pi_pico — UART command maker_pi_pico
# ==============================================================================
add_executable(maker_pi_pico
    src/maker_pi_pico.c
    src/uart_comm.c
)

target_include_directories(maker_pi_pico PRIVATE ${PROJ_INCLUDE_DIRS})

target_link_libraries(maker_pi_pico
    pico_stdlib
    hardware_uart
    hardware_gpio
)

pico_enable_stdio_usb(maker_pi_pico 1)
pico_enable_stdio_uart(maker_pi_pico 0)
pico_add_extra_outputs(maker_pi_pico)

# ==============================================================================
# TARGET 3: drift_correction — Calibrated navigation with encoders + IMU
# ==============================================================================
add_executable(drift_correction
    src/drift_correction.c
    src/motors.c
    src/imu.c
    src/encoder.c
)

target_include_directories(drift_correction PRIVATE ${PROJ_INCLUDE_DIRS})

target_link_libraries(drift_correction
    pico_stdlib
    hardware_i2c
    hardware_pwm
    hardware_gpio
)

pico_enable_stdio_usb(drift_correction 1)
pico_enable_stdio_uart(drift_correction 0)
pico_add_extra_outputs(drift_correction)

# ==============================================================================
# TARGET 4: auto_turn_test — Automatic turn test sequence (battery operation)
# ==============================================================================
add_executable(auto_turn_test
    src/auto_turn_test.c
    src/motors.c
    src/imu.c
    src/encoder.c
)

target_include_directories(auto_turn_test PRIVATE ${PROJ_INCLUDE_DIRS})

target_link_libraries(auto_turn_test
    pico_stdlib
    hardware_i2c
    hardware_pwm
    hardware_gpio
)

pico_enable_stdio_usb(auto_turn_test 0)
pico_enable_stdio_uart(auto_turn_test 0)
pico_add_extra_outputs(auto_turn_test)

# ==============================================================================
# TARGET 5: main — (NEW) Cascaded IMU + Encoder Control
# IMPORTANT: Save the code we just wrote as 'src/main.c'
# ==============================================================================
add_executable(main
    src/main.c
    src/motors.c
    src/imu.c
    src/encoder.c
)

target_include_directories(main PRIVATE ${PROJ_INCLUDE_DIRS})

target_link_libraries(main
    pico_stdlib
    hardware_i2c
    hardware_pwm
    hardware_gpio
    hardware_timer
)

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)
pico_add_extra_outputs(main)