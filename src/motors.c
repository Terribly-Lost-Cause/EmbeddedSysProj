#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "motors.h"

/* =============================================================================
 *  File: motors.c
 *  Purpose:
 *    - Low-level motor driver for a 2-wheel differential drive (L/R wheels)
 *    - Provides: init, stop, forward, backward, left, right, set/get speed
 *    - Uses PWM on 4 GPIOs (L_IN1, L_IN2, R_IN1, R_IN2) defined in motors.h
 *
 *  Key behavior:
 *    - Speed is a percentage (clamped 40–100%) to guarantee reliable start
 *    - Direction is set by which input (IN1 vs IN2) gets the PWM
 *    - LEFT/RIGHT are defined as in-place pivots:
 *        LEFT  = left wheel forward, right wheel backward
 *        RIGHT = left wheel backward, right wheel forward
 *      (This matches your chassis so that 'A' pivots left and 'D' pivots right.)
 *
 * =============================================================================
 */

/* ----------------------------- Internal State ------------------------------ */

// Motion state so that a speed change immediately re-applies to current motion.
typedef enum {
    STATE_STOP = 0,
    STATE_FWD,
    STATE_BACK,
    STATE_LEFT,
    STATE_RIGHT
} motion_state_t;

static motion_state_t g_state = STATE_STOP;

// Current speed percentage (clamped to 40..100)
static uint8_t g_speed_pct = 60;   // default 60% at power-up

/* ----------------------------- PWM Utilities --------------------------------
 * Each motor input pin is driven by a PWM slice. We:
 *   1) set the pin to PWM function
 *   2) configure wrap/clkdiv
 *   3) enable the slice
 *   4) write duty (0..255) based on requested percentage
 * ---------------------------------------------------------------------------*/

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
    // Map 0..100% -> 0..255
    uint16_t level = (uint16_t)((pct * 255u) / 100u);
    pwm_set_gpio_level(gpio, level);
}

/* --------------------------- Apply Current State ----------------------------
 * This is the single place that sets the 4 PWM outputs according to:
 *   - current motion state (STOP/FWD/BACK/LEFT/RIGHT)
 *   - current speed percentage (g_speed_pct)
 * We call this every time state or speed changes.
 * ---------------------------------------------------------------------------*/

static void apply_state(void) {
    switch (g_state) {
        case STATE_STOP:
            pwm_set_pct(L_IN1, 0);
            pwm_set_pct(L_IN2, 0);
            pwm_set_pct(R_IN1, 0);
            pwm_set_pct(R_IN2, 0);
            break;

        case STATE_FWD:
            // Both wheels forward: IN1 = speed, IN2 = 0
            pwm_set_pct(L_IN1, g_speed_pct); pwm_set_pct(L_IN2, 0);
            pwm_set_pct(R_IN1, g_speed_pct); pwm_set_pct(R_IN2, 0);
            break;

        case STATE_BACK:
            // Both wheels backward: IN1 = 0, IN2 = speed
            pwm_set_pct(L_IN1, 0);           pwm_set_pct(L_IN2, g_speed_pct);
            pwm_set_pct(R_IN1, 0);           pwm_set_pct(R_IN2, g_speed_pct);
            break;

        case STATE_LEFT:
            // Pivot LEFT: left wheel forward, right wheel backward
            // Matches your desired A=LEFT behavior
            pwm_set_pct(L_IN1, g_speed_pct); pwm_set_pct(L_IN2, 0);
            pwm_set_pct(R_IN1, 0);           pwm_set_pct(R_IN2, g_speed_pct);
            break;

        case STATE_RIGHT:
            // Pivot RIGHT: left wheel backward, right wheel forward
            // Matches your desired D=RIGHT behavior
            pwm_set_pct(L_IN1, 0);           pwm_set_pct(L_IN2, g_speed_pct);
            pwm_set_pct(R_IN1, g_speed_pct); pwm_set_pct(R_IN2, 0);
            break;
    }
}

/* =============================== Public API =================================
 * The following functions are called by main.c based on user/controller input.
 * They update internal state and immediately apply PWM outputs.
 * ===========================================================================*/

void motors_init(void) {
    // Configure PWM on all four motor inputs
    pwm_setup_gpio(L_IN1);
    pwm_setup_gpio(L_IN2);
    pwm_setup_gpio(R_IN1);
    pwm_setup_gpio(R_IN2);

    // Start in a known safe state
    g_state = STATE_STOP;
    apply_state();
}

void motors_set_speed(uint8_t pct) {
    // Clamp to 40..100% for reliable start on L298N + DC gearmotors
    if (pct < 40) pct = 40;
    if (pct > 100) pct = 100;

    g_speed_pct = pct;
    apply_state();   // Re-apply current motion with new speed
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
    apply_state();
}

void motors_right(void) {
    g_state = STATE_RIGHT;
    apply_state();
}


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

