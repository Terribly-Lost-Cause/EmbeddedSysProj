#pragma once
#include "pico/stdlib.h"

void imu_init(void);
void imu_read(float *heading, float *roll, float *pitch);
