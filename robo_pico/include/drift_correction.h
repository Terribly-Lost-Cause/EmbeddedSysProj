#ifndef DRIFT_CORRECTION_H
#define DRIFT_CORRECTION_H

#include <stdbool.h>
#include <stdint.h>

// Initialize the drift correction system.
// Sets up motors, encoders, resets counters.
void drift_correction_init(void);

// Start matching the encoders.
// Call this when you begin moving forward.
void drift_correction_start(void);

// Stop matching and stop motors.
// Call this when forward movement ends.
void drift_correction_stop(void);

// Perform one update cycle (every 10ms).
// Reads both encoders, compares pulses,
// adjusts L/R motor bias using motors_forward_bias().
bool drift_correction_update(void);

// Print debug info for L/R pulses and bias.
// Optional, only for testing.
void drift_correction_print_status(void);



// --- OPTIONAL (USED ONLY BY AUTO TEST MODE IN drift_correction.c) ---
void auto_test_init(void);
bool auto_test_update(void);

#endif // DRIFT_CORRECTION_H
