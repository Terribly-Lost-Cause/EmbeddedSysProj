#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"

// =======================================
// GLOBAL ENCODER PIN DEFINITIONS
// =======================================
#define LEFT_ENCODER_PIN   2
#define RIGHT_ENCODER_PIN  6

// =======================================
// ROBOT POSE STRUCT
// =======================================
typedef struct {
    float x;
    float y;
    float theta;
} Pose;

// =======================================
// FUNCTION DECLARATIONS
// =======================================
void encoder_init(uint32_t encoder_pin);

bool encoder_check_pulse(uint32_t pin);

float calculate_rpm(uint32_t pulses,
                    uint32_t pulses_per_rev,
                    float interval_sec);

float distance_from_pulses(int pulses,
                           int pulses_per_rev,
                           float wheel_radius);

void update_odometry(Pose *pose,
                     int pulses_left,
                     int pulses_right,
                     int pulses_per_rev,
                     float wheel_radius,
                     float wheel_base);

#endif
