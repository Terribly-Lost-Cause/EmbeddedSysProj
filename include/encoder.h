#ifndef ENCODER_H
#define ENCODER_H

// --- Standard Libraries ---
// These give us access to basic types like 'uint32_t' (numbers) and 'bool' (true/false)
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"

// --- Available Tools ---

// Prepare a specific pin on the Pico to read sensor data
void encoder_init(uint32_t encoder_pin);

// Check if the wheel just passed a slot in the encoder disk
// Returns 'true' only at the exact moment a pulse is detected
bool encoder_check_pulse(uint32_t encoder_pin);

// Calculate the speed of the wheel
// Converts raw "clicks" into "Revolutions Per Minute" (RPM)
float calculate_rpm(uint32_t pulses, uint32_t pulses_per_rev, float interval_sec);

#endif