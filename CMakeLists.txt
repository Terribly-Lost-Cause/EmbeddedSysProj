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

add_executable(drift_correction
    src/drift_correction.c
    src/encoder.c
    src/imu.c
    src/motors.c
)

target_include_directories(drift_correction PRIVATE ${PROJ_INCLUDE_DIR})

target_link_libraries(drift_correction
    pico_stdlib
    hardware_i2c
    hardware_pwm
)

pico_enable_stdio_usb(drift_correction 1)
pico_enable_stdio_uart(drift_correction 0)

pico_add_extra_outputs(drift_correction)