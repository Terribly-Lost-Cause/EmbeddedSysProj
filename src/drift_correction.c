#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "encoder.h"
#include "motors.h"
#include "imu.h"

#define LEFT_ENCODER_PIN 2
#define RIGHT_ENCODER_PIN 6
#define PULSES_PER_REV 20
#define CONTROL_LOOP_MS 100
#define DEFAULT_SPEED 40
#define MAX_BIAS 6
#define MAX_AUTO_BIAS 5
#define KP_HEADING 0.18f
#define DEAD_BAND_HEADING 6.0f
#define IMU_WINDOW 7                    // Number of samples for heading median filter
#define HEADING_JUMP_MAX 45.0f          // Max IMU jump allowed between updates
#define BIAS_LEARN_INTERVAL 100         // Encoder adaptive bias update interval
#define BIAS_LEARN_STEP 1               // Per-interval bias update amount
#define BIAS_LEARN_LIMIT 5              // Max persistent bias value

typedef struct {
    float target_heading;
    float current_heading;
    float heading_error;
    int8_t left_bias;
    int8_t right_bias;
    uint32_t left_pulses;
    uint32_t right_pulses;
    bool active;
} drift_correction_t;

static drift_correction_t dc = {0};
static int32_t prev_left_pulses = 0;
static int32_t prev_right_pulses = 0;

// Stores recent IMU heading values for filtering
static float heading_buffer[IMU_WINDOW] = {0};
static int heading_idx = 0;

// Persistent variables for adaptive encoder bias
static int persistent_bias = 0;
static int encoder_diff_accum = 0;
static int encoder_update_count = 0;

// Compare two float values (for sorting)
int compare_floats(const void *a, const void *b) {
    float fa = *(const float *)a, fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

// Returns median heading from the buffer with new value added
float imu_median(float new_heading) {
    heading_buffer[heading_idx] = new_heading;
    heading_idx = (heading_idx + 1) % IMU_WINDOW;
    float temp[IMU_WINDOW];
    for (int i = 0; i < IMU_WINDOW; ++i) temp[i] = heading_buffer[i];
    qsort(temp, IMU_WINDOW, sizeof(float), compare_floats);
    return temp[IMU_WINDOW / 2];
}

// Normalizes angle to [0, 360)
static float normalize_angle(float angle) {
    while (angle < 0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

// Calculates signed shortest angle difference between two headings
static float heading_difference(float target, float current) {
    float diff = target - current;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

// Clamps value between min and max
static int clamp(int value, int min, int max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// Initialize hardware and all drift correction variables
void drift_correction_init(void) {
    printf("Initializing drift correction system...\n");
    encoder_init(LEFT_ENCODER_PIN);
    encoder_init(RIGHT_ENCODER_PIN);
    imu_init();
    motors_init();
    dc.target_heading = dc.current_heading = dc.heading_error = 0.0f;
    dc.left_bias = dc.right_bias = dc.left_pulses = dc.right_pulses = dc.active = 0;
    prev_left_pulses = prev_right_pulses = 0;
    for (int i = 0; i < IMU_WINDOW; ++i) heading_buffer[i] = 0;
    heading_idx = 0;
    motors_set_speed(DEFAULT_SPEED);
    printf(" Encoders: GP%d (L), GP%d (R)\n", LEFT_ENCODER_PIN, RIGHT_ENCODER_PIN);
    printf(" IMU: Initialized for heading correction\n");
    printf("Drift correction ready!\n\n");
}

// Start drift correction with current heading as the baseline
void drift_correction_start(void) {
    float heading, roll, pitch;
    imu_read(&heading, &roll, &pitch);
    dc.target_heading = normalize_angle(heading);
    for (int i = 0; i < IMU_WINDOW; ++i) heading_buffer[i] = dc.target_heading;
    heading_idx = 0;
    dc.current_heading = dc.target_heading;
    dc.heading_error = 0.0f;
    dc.left_bias = dc.right_bias = dc.left_pulses = dc.right_pulses = 0;
    dc.active = 1;
    prev_left_pulses = prev_right_pulses = 0;
    persistent_bias = encoder_diff_accum = encoder_update_count = 0;
    printf("Drift correction STARTED - Target heading: %.2f°\n", dc.target_heading);
}

// Stop drift correction and reset biases
void drift_correction_stop(void) {
    dc.active = false;
    dc.left_bias = dc.right_bias = 0;
    printf("Drift correction STOPPED\n");
}

// Main update loop: one run for each control cycle
bool drift_correction_update(void) {
    if (!dc.active) return false;

    // --- Get filtered heading ---
    float heading, roll, pitch;
    imu_read(&heading, &roll, &pitch);
    float med_heading = imu_median(normalize_angle(heading));
    float heading_step = fabs(med_heading - dc.current_heading);
    if (heading_step > 180.0f) heading_step = 360.0f - heading_step;
    if (heading_step < HEADING_JUMP_MAX) dc.current_heading = med_heading;

    // --- Calculate heading correction ---
    dc.heading_error = heading_difference(dc.target_heading, dc.current_heading);
    float filtered_error = (fabs(dc.heading_error) >= DEAD_BAND_HEADING) ? dc.heading_error : 0.0f;
    int heading_bias = clamp((int)(KP_HEADING * filtered_error), -MAX_BIAS, MAX_BIAS);

    // --- Check encoder pulses and adjust biases ---
    if (encoder_check_pulse(LEFT_ENCODER_PIN)) dc.left_pulses++;
    if (encoder_check_pulse(RIGHT_ENCODER_PIN)) dc.right_pulses++;
    int left_delta = dc.left_pulses - prev_left_pulses, right_delta = dc.right_pulses - prev_right_pulses;
    int auto_error = left_delta - right_delta;
    int auto_bias = clamp(auto_error, -MAX_AUTO_BIAS, MAX_AUTO_BIAS);

    // --- Learn persistent encoder bias (long-term) ---
    encoder_diff_accum += (int)dc.left_pulses - (int)dc.right_pulses;
    encoder_update_count++;
    if (encoder_update_count >= BIAS_LEARN_INTERVAL) {
        if (encoder_diff_accum > BIAS_LEARN_INTERVAL)
            persistent_bias = clamp(persistent_bias - BIAS_LEARN_STEP, -BIAS_LEARN_LIMIT, BIAS_LEARN_LIMIT);
        else if (encoder_diff_accum < -BIAS_LEARN_INTERVAL)
            persistent_bias = clamp(persistent_bias + BIAS_LEARN_STEP, -BIAS_LEARN_LIMIT, BIAS_LEARN_LIMIT);
        encoder_diff_accum = 0;
        encoder_update_count = 0;
    }

    // --- Combine all bias terms and set motor output ---
    dc.left_bias = heading_bias - auto_bias + persistent_bias;
    dc.right_bias = -heading_bias + auto_bias - persistent_bias;
    motors_forward_bias(dc.left_bias, dc.right_bias);

    prev_left_pulses = dc.left_pulses;
    prev_right_pulses = dc.right_pulses;
    return true;
}

// Print current correction status to the terminal
void drift_correction_print_status(void) {
    if (!dc.active) { printf("Drift correction: INACTIVE\n"); return; }
    printf("Target: %6.2f | Current: %6.2f | Error: %+6.2f | Bias: L%+3d R%+3d | Enc: L%5d R%5d | Off %+2d\n",
        dc.target_heading, dc.current_heading, dc.heading_error,
        dc.left_bias, dc.right_bias, dc.left_pulses, dc.right_pulses, persistent_bias);
}

int main() {
    stdio_init_all();
    sleep_ms(3000);

    printf("\n\n╔════════════════════════════════════════════════════════════╗\n");
    printf("║ ROBOT DRIFT CORRECTION SYSTEM - IMU+Encoder ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n\n");

    drift_correction_init();

    printf("Commands:\n W - Start forward with drift correction\n S - Stop motors\n + - Increase speed by 5%%\n - - Decrease speed by 5%%\n\nReady! Press 'W' to start...\n\n");

    absolute_time_t last_control_update = get_absolute_time();
    absolute_time_t last_status_print = get_absolute_time();
    bool motors_running = false;

    while (true) {
        int ch = getchar_timeout_us(0);
        if (ch != PICO_ERROR_TIMEOUT) {
            switch (ch) {
                case 'w': case 'W':
                    if (!motors_running) {
                        motors_forward();
                        drift_correction_start();
                        motors_running = true;
                        printf("\n>>> FORWARD with drift correction <<<\n\n");
                    }
                    break;
                case 's': case 'S':
                    motors_stop();
                    drift_correction_stop();
                    motors_running = false;
                    printf("\n>>> STOPPED <<<\n\n");
                    break;
                case '+': case '=': {
                    uint8_t speed = motors_get_speed();
                    motors_set_speed(speed + 5);
                    printf("Speed: %d%%\n", motors_get_speed());
                }
                break;
                case '-': case '_': {
                    uint8_t speed = motors_get_speed();
                    motors_set_speed(speed - 5);
                    printf("Speed: %d%%\n", motors_get_speed());
                }
                break;
            }
        }
        absolute_time_t now = get_absolute_time();
        int64_t control_elapsed_us = absolute_time_diff_us(last_control_update, now);
        if (control_elapsed_us >= CONTROL_LOOP_MS * 1000) {
            drift_correction_update();
            last_control_update = now;
        }
        int64_t status_elapsed_us = absolute_time_diff_us(last_status_print, now);
        if (status_elapsed_us >= 200000) {
            if (motors_running) drift_correction_print_status();
            last_status_print = now;
        }
    }
}
