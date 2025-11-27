#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "encoder.h"
#include "motors.h"

// --- Configuration ---
#define LEFT_ENCODER_PIN 2
#define RIGHT_ENCODER_PIN 6
#define PULSES_PER_REV 20
#define CONTROL_LOOP_MS 10
#define CALIBRATION_SPEED 30
#define DEFAULT_SPEED 30

// --- Auto Test Configuration ---
#define AUTO_START_DELAY_MS 3000   // Wait 3 seconds before starting
#define AUTO_RUN_DURATION_MS 10000 // Run for 10 seconds
#define AUTO_STOP_DURATION_MS 5000 // Stop for 5 seconds before next run
#define AUTO_TEST_CYCLES 1         // Number of test cycles (changed from 3 to 1)

// --- Aggressive Correction ---
#define MAX_CORRECTION 15

// --- Robot State ---
typedef struct {
    int8_t left_bias;
    int8_t right_bias;
    uint32_t left_pulses;
    uint32_t right_pulses;
    bool active;
} drift_correction_t;

typedef enum {
    STATE_IDLE,
    STATE_WAITING_TO_START,
    STATE_RUNNING,
    STATE_STOPPED,
    STATE_COMPLETE
} test_state_t;

typedef struct {
    test_state_t state;
    uint32_t cycle_count;
    absolute_time_t state_start_time;
    bool auto_mode_enabled;
} auto_test_t;

static drift_correction_t dc = {0};
static auto_test_t test = {STATE_IDLE, 0, {0}, true};

static int clamp(int value, int min, int max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// --- System Setup ---
void drift_correction_init(void) {
    printf("Initializing STRICT encoder matching system...\n");
    encoder_init(LEFT_ENCODER_PIN);
    encoder_init(RIGHT_ENCODER_PIN);
    motors_init();
    dc.left_bias = dc.right_bias = dc.left_pulses = dc.right_pulses = dc.active = 0;
    motors_set_speed(DEFAULT_SPEED);
    printf("  Mode: ZERO TOLERANCE - Encoders MUST match\n");
    printf("  Max Correction: ±%d\n", MAX_CORRECTION);
    printf("Ready!\n\n");
}

// --- Start Driving ---
void drift_correction_start(void) {
    dc.left_bias = dc.right_bias = dc.left_pulses = dc.right_pulses = 0;
    dc.active = 1;
    printf("STARTED - STRICT encoder matching ACTIVE\n");
}

void drift_correction_stop(void) {
    dc.active = false;
    dc.left_bias = dc.right_bias = 0;
    motors_stop();
    printf("STOPPED\n");
}

// --- STRICT Encoder Matching - ZERO DIFFERENCE ALLOWED ---
bool drift_correction_update(void) {
    if (!dc.active) return false;
    
    // 1. Read Encoders
    if (encoder_check_pulse(LEFT_ENCODER_PIN)) dc.left_pulses++;
    if (encoder_check_pulse(RIGHT_ENCODER_PIN)) dc.right_pulses++;
    
    // 2. Calculate difference
    int32_t diff = (int32_t)dc.left_pulses - (int32_t)dc.right_pulses;
    
    // 3. AGGRESSIVE correction to force zero difference
    if (diff > 0) {
        dc.left_bias = -MAX_CORRECTION;
        dc.right_bias = +MAX_CORRECTION;
    } else if (diff < 0) {
        dc.left_bias = +MAX_CORRECTION;
        dc.right_bias = -MAX_CORRECTION;
    } else {
        dc.left_bias = 0;
        dc.right_bias = 0;
    }
    
    // 4. Apply corrections immediately
    motors_forward_bias(dc.left_bias, dc.right_bias);
    return true;
}

void drift_correction_print_status(void) {
    if (!dc.active) {
        printf("Encoder matching: INACTIVE\n");
        return;
    }
    
    int32_t diff = (int32_t)dc.left_pulses - (int32_t)dc.right_pulses;
    const char* status_msg = " ✓✓✓ PERFECT MATCH ✓✓✓ ";
    if (diff > 0) {
        status_msg = "!! LEFT AHEAD - MAX CORRECTION !!";
    } else if (diff < 0) {
        status_msg = "!! RIGHT AHEAD - MAX CORRECTION !!";
    }
    
    printf("L:%lu R:%lu Diff:%+ld | Bias: L%+d R%+d | %s\n",
           dc.left_pulses, dc.right_pulses, diff,
           dc.left_bias, dc.right_bias, status_msg);
}

// --- AUTO TEST STATE MACHINE ---
void auto_test_init(void) {
    test.state = STATE_WAITING_TO_START;
    test.cycle_count = 0;
    test.state_start_time = get_absolute_time();
    printf("\n>>> AUTO TEST MODE <<<\n");
    printf("Starting in %d seconds...\n", AUTO_START_DELAY_MS / 1000);
    printf("Test duration: %ds RUN\n", AUTO_RUN_DURATION_MS / 1000);
    printf("Total cycles: 1\n\n");
}

bool auto_test_update(void) {
    absolute_time_t now = get_absolute_time();
    int64_t elapsed_ms = absolute_time_diff_us(test.state_start_time, now) / 1000;
    
    switch (test.state) {
        case STATE_WAITING_TO_START:
            if (elapsed_ms >= AUTO_START_DELAY_MS) {
                test.cycle_count++;
                printf("\n╔════════════════════════════════════════════════╗\n");
                printf("║  STARTING FORWARD MOTION                       ║\n");
                printf("╚════════════════════════════════════════════════╝\n\n");
                motors_forward();
                drift_correction_start();
                test.state = STATE_RUNNING;
                test.state_start_time = now;
            }
            break;
            
        case STATE_RUNNING:
            if (elapsed_ms >= AUTO_RUN_DURATION_MS) {
                motors_stop();
                drift_correction_stop();
                printf("\n>>> TEST COMPLETE <<<\n");
                printf("Final: L:%lu R:%lu | Diff:%ld pulses\n",
                       dc.left_pulses, dc.right_pulses,
                       (int32_t)(dc.left_pulses - dc.right_pulses));
                
                test.state = STATE_COMPLETE;
                printf("\n╔════════════════════════════════════════════════╗\n");
                printf("║           TEST CYCLE COMPLETE                  ║\n");
                printf("╚════════════════════════════════════════════════╝\n\n");
                return false; // Signal completion
            }
            break;
            
        case STATE_COMPLETE:
            return false;
            
        default:
            break;
    }
    
    return true;
}

// ═══════════════════════════════════════════════════════════
// ║                      MAIN PROGRAM                       ║
// ═══════════════════════════════════════════════════════════
int main() {
    stdio_init_all();
    sleep_ms(3000);
    
    printf("\n\n╔════════════════════════════════════════════════════════════╗\n");
    printf("║         STRICT ENCODER MATCHING CONTROLLER                ║\n");
    printf("║               AUTOMATIC TEST MODE                          ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n\n");
    
    drift_correction_init();
    auto_test_init();
    
    absolute_time_t last_control_update = get_absolute_time();
    absolute_time_t last_status_print = get_absolute_time();
    
    while (true) {
        // Manual override always available
        int ch = getchar_timeout_us(0);
        if (ch != PICO_ERROR_TIMEOUT) {
            switch (ch) {
                case 'w': case 'W':
                    test.state = STATE_IDLE; // Disable auto mode
                    test.auto_mode_enabled = false;
                    motors_forward();
                    drift_correction_start();
                    printf("\n>>> MANUAL FORWARD <<<\n\n");
                    break;
                case 's': case 'S':
                    test.state = STATE_IDLE;
                    test.auto_mode_enabled = false;
                    motors_stop();
                    drift_correction_stop();
                    printf("\n>>> MANUAL STOP <<<\n");
                    printf("Final: L:%lu R:%lu | Diff:%ld pulses\n\n",
                           dc.left_pulses, dc.right_pulses,
                           (int32_t)(dc.left_pulses - dc.right_pulses));
                    break;
            }
        }
        
        absolute_time_t now = get_absolute_time();
        
        // Auto test state machine
        if (test.auto_mode_enabled && test.state != STATE_IDLE) {
            if (!auto_test_update()) {
                // Test complete - disable auto mode
                test.auto_mode_enabled = false;
            }
        }
        
        // Update correction VERY frequently (every 10ms)
        int64_t control_elapsed_us = absolute_time_diff_us(last_control_update, now);
        if (control_elapsed_us >= CONTROL_LOOP_MS * 1000) {
            drift_correction_update();
            last_control_update = now;
        }
        
        // Print status every 300ms
        int64_t status_elapsed_us = absolute_time_diff_us(last_status_print, now);
        if (status_elapsed_us >= 300000) {
            if (dc.active) drift_correction_print_status();
            last_status_print = now;
        }
    }
}
