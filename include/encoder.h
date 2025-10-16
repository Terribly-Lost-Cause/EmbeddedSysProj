#ifndef ENCODER_H
#define ENCODER_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"

void encoder_init(uint32_t encoder_pin);
bool encoder_check_pulse(uint32_t encoder_pin);
float calculate_rpm(uint32_t pulses, uint32_t pulses_per_rev, float interval_sec);

#endif
