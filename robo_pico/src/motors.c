#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "motors.h"
#include "imu.h"
#include <math.h>
#include <stdlib.h>

// --- Definitions ---
typedef enum {
    STATE_STOP = 0,
    STATE_FWD,
    STATE_BACK,
    STATE_LEFT,
    STATE_RIGHT
} motion_state_t;

static motion_state_t g_state = STATE_STOP;
static uint8_t g_speed_pct = 10;

// Variables for handling precise turns
static bool turning_active = false;
static bool turn_direction_left = false;  // Track which direction we're turning
static float turn_start_heading = 0.0f;
static float turn_target_heading = 0.0f;

// --- Noise Filtering Settings ---
#define TURN_FILTER_SIZE 9
static float turn_heading_buffer[TURN_FILTER_SIZE];
static int turn_filter_idx = 0;
static bool turn_filter_filled = false;

// --- Motor Power Control (PWM) ---
static inline void pwm_setup_gpio(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice, 255);
    pwm_set_clkdiv(slice, 24.0f);
    pwm_set_enabled(slice, true);
    pwm_set_gpio_level(gpio, 0);
}

static inline void pwm_set_pct(uint gpio, uint8_t pct) {
    if (pct > 100) pct = 100;
    uint16_t level = (uint16_t)((pct * 255u) / 100u);
    pwm_set_gpio_level(gpio, level);
}

static void apply_state(void) {
    switch (g_state) {
        case STATE_STOP:
            pwm_set_pct(L_IN1, 0); pwm_set_pct(L_IN2, 0);
            pwm_set_pct(R_IN1, 0); pwm_set_pct(R_IN2, 0);
            break;
        case STATE_FWD:
            pwm_set_pct(L_IN1, g_speed_pct); pwm_set_pct(L_IN2, 0);
            pwm_set_pct(R_IN1, g_speed_pct); pwm_set_pct(R_IN2, 0);
            break;
        case STATE_BACK:
            pwm_set_pct(L_IN1, 0); pwm_set_pct(L_IN2, g_speed_pct);
            pwm_set_pct(R_IN1, 0); pwm_set_pct(R_IN2, g_speed_pct);
            break;
        case STATE_LEFT:
            pwm_set_pct(L_IN1, g_speed_pct); pwm_set_pct(L_IN2, 0);
            pwm_set_pct(R_IN1, 0); pwm_set_pct(R_IN2, g_speed_pct);
            break;
        case STATE_RIGHT:
            pwm_set_pct(L_IN1, 0); pwm_set_pct(L_IN2, g_speed_pct);
            pwm_set_pct(R_IN1, g_speed_pct); pwm_set_pct(R_IN2, 0);
            break;
    }
}

// --- Math Helpers ---
static float normalize_angle(float angle) {
    while (angle < 0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

static float angle_difference(float target, float current) {
    float diff = target - current;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

static int compare_floats(const void *a, const void *b) {
    float fa = *(const float*)a;
    float fb = *(const float*)b;
    return (fa > fb) - (fa < fb);
}

static float get_filtered_heading(void) {
    float heading, roll, pitch;
    imu_read(&heading, &roll, &pitch);
    turn_heading_buffer[turn_filter_idx] = normalize_angle(heading);
    turn_filter_idx = (turn_filter_idx + 1) % TURN_FILTER_SIZE;
    if (turn_filter_idx == 0) turn_filter_filled = true;
    
    if (!turn_filter_filled) {
        return normalize_angle(heading);
    }
    
    float temp[TURN_FILTER_SIZE];
    for (int i = 0; i < TURN_FILTER_SIZE; i++) {
        temp[i] = turn_heading_buffer[i];
    }
    
    qsort(temp, TURN_FILTER_SIZE, sizeof(float), compare_floats);
    return temp[TURN_FILTER_SIZE / 2];
}

// --- Public Functions ---
void motors_init(void) {
    pwm_setup_gpio(L_IN1);
    pwm_setup_gpio(L_IN2);
    pwm_setup_gpio(R_IN1);
    pwm_setup_gpio(R_IN2);
    g_state = STATE_STOP;
    apply_state();
    
    for (int i = 0; i < TURN_FILTER_SIZE; i++) {
        turn_heading_buffer[i] = 0.0f;
    }
    turn_filter_idx = 0;
    turn_filter_filled = false;
}

void motors_set_speed(uint8_t pct) {
    if (pct < 10) pct = 10;
    if (pct > 100) pct = 100;
    g_speed_pct = pct;
    apply_state();
}

uint8_t motors_get_speed(void) {
    return g_speed_pct;
}

void motors_stop(void) {
    g_state = STATE_STOP;
    apply_state();
}

void motors_forward(void) {
    g_state = STATE_FWD;
    apply_state();
}

void motors_backward(void) {
    g_state = STATE_BACK;
    apply_state();
}

void motors_left(void) {
    g_state = STATE_LEFT;
    // Use equal speed for both motors, minimum 35% for reliable turns
    uint8_t turn_speed = (g_speed_pct < 35) ? 35 : g_speed_pct;
    pwm_set_pct(L_IN1, turn_speed);
    pwm_set_pct(L_IN2, 0);
    pwm_set_pct(R_IN1, 0);
    pwm_set_pct(R_IN2, turn_speed);
}

void motors_right(void) {
    g_state = STATE_RIGHT;
    // Use equal speed for both motors, minimum 35% for reliable turns
    uint8_t turn_speed = (g_speed_pct < 35) ? 35 : g_speed_pct;
    pwm_set_pct(L_IN1, 0);
    pwm_set_pct(L_IN2, turn_speed);
    pwm_set_pct(R_IN1, turn_speed);
    pwm_set_pct(R_IN2, 0);
}

// Apply bias on top of standard speed for drift correction
void motors_forward_bias(int8_t left_bias, int8_t right_bias) {
    int l_pct = g_speed_pct + left_bias;
    int r_pct = g_speed_pct + right_bias;
    
    if (l_pct < 0) l_pct = 0;
    if (l_pct > 100) l_pct = 100;
    if (r_pct < 0) r_pct = 0;
    if (r_pct > 100) r_pct = 100;
    
    pwm_set_pct(L_IN1, l_pct); pwm_set_pct(L_IN2, 0);
    pwm_set_pct(R_IN1, r_pct); pwm_set_pct(R_IN2, 0);
}

// --- Smart Turning Logic with Speed Ramping ---
void motors_turn_to_heading(float target_heading, bool turn_left) {
    turn_filter_idx = 0;
    turn_filter_filled = false;
    for (int i = 0; i < TURN_FILTER_SIZE; i++) {
        float h, r, p;
        imu_read(&h, &r, &p);
        turn_heading_buffer[i] = normalize_angle(h);
        sleep_ms(30);
    }
    
    turn_filter_filled = true;
    float filtered_heading = get_filtered_heading();
    turn_start_heading = filtered_heading;
    turn_target_heading = normalize_angle(target_heading);
    turning_active = true;
    turn_direction_left = turn_left;
    
    // Start at full speed
    motors_set_speed(35);
    
    if (turn_left) {
        motors_left();
    } else {
        motors_right();
    }
}

bool motors_is_turning(void) {
    if (!turning_active) return false;
    
    float current = get_filtered_heading();
    float error = fabsf(angle_difference(turn_target_heading, current));
    
    // AGGRESSIVE multi-stage speed control for maximum precision
    if (error < 1.5f) {
        // Target reached - stop immediately
        motors_stop();
        turning_active = false;
        return false;
    } else if (error < 5.0f) {
        // Very close - crawl speed for fine adjustment
        motors_set_speed(18);
        if (turn_direction_left) {
            motors_left();
        } else {
            motors_right();
        }
    } else if (error < 15.0f) {
        // Close - slow speed
        motors_set_speed(25);
        if (turn_direction_left) {
            motors_left();
        } else {
            motors_right();
        }
    } else if (error < 30.0f) {
        // Medium distance - moderate speed
        motors_set_speed(30);
        if (turn_direction_left) {
            motors_left();
        } else {
            motors_right();
        }
    } else {
        // Far from target - full speed
        motors_set_speed(35);
        if (turn_direction_left) {
            motors_left();
        } else {
            motors_right();
        }
    }
    
    return true;
}
