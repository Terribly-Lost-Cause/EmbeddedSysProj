#pragma once

#include "pico/stdlib.h"

// --- Wiring Setup ---
#define L_IN1 10 
#define L_IN2 11 
#define R_IN1 8  
#define R_IN2 9  

void motors_init(void);
void motors_stop(void);
void motors_forward(void);
void motors_backward(void);
void motors_left(void);  
void motors_right(void); 

// --- Speed Control ---
void motors_set_speed(uint8_t pct);

// Check what the current speed setting is
uint8_t motors_get_speed(void);

void motors_forward_bias(int8_t left_bias, int8_t right_bias);
void motors_turn_to_heading(float target_heading, bool turn_left);

bool motors_is_turning(void);
