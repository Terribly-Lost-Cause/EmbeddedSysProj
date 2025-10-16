#pragma once
#include "pico/stdlib.h"

// Motor control pins
#define L_IN1 8   // Left motor input 1  (GP8)
#define L_IN2 9   // Left motor input 2  (GP9)
#define R_IN1 10  // Right motor input 1 (GP10)
#define R_IN2 11  // Right motor input 2 (GP11)

void motors_init(void);
void motors_stop(void);
void motors_forward(void);
void motors_backward(void);
void motors_left(void);
void motors_right(void);

// Speed control (0-100%)
void motors_set_speed(uint8_t pct);
uint8_t motors_get_speed(void);