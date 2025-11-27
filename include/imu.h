#pragma once

#include "pico/stdlib.h"

// Initialize the IMU (LSM303DLHC)
void imu_init(void);

// Calibrate magnetometer and accelerometer
// Robot must be rotated in all directions during calibration
void imu_calibrate(void);

// Read calibrated compass heading and tilt
// heading = 0-360° compass direction
// roll/pitch = tilt angles in degrees
void imu_read(float *heading, float *roll, float *pitch);

// Get raw magnetometer readings (for calibration verification)
void imu_read_raw_mag(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z);
