/*
================================================================================
 File:        main_robot.c
 Purpose:     Unified runtime for the robot: Motors + Encoders + IMU + Kill Switch
 Output:      Minimal telemetry line every SAMPLE_MS, plus "Drift detected"/"cleared"

 QUICK START (where to edit)
 ────────────────────────────────────────────────────────────────────────────────
 [A] CONFIG             → Pins, pulses-per-rev, sample rate
 [B] THRESHOLDS         → Drift sensitivity (RPM delta, Heading delta)
 [C] STARTUP BEHAVIOR   → Default speed and initial motion (forward/stop/backward)
 [D] TELEMETRY FORMAT   → One-line printf, interval
 [E] EXTENSIONS         → Hooks to add features (auto-correct drift, logging, etc.)

 BUILD
 ────────────────────────────────────────────────────────────────────────────────
 - Target name: robot (see CMakeLists.txt)
 - Links: pico_stdlib, hardware_pwm, hardware_i2c, hardware_gpio

 DEPENDENCIES (feature modules; keep logic inside them)
 ────────────────────────────────────────────────────────────────────────────────
 - motors.c/.h   : motors_init(), motors_forward(), motors_stop(), motors_set_speed(), motors_get_speed()
 - encoder.c/.h  : encoder_init(pin), encoder_check_pulse(pin), calculate_rpm(pulses, ppr, dt)
 - imu.c/.h      : imu_init(), imu_read(&heading,&roll,&pitch)
================================================================================
*/

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "motors.h"
#include "encoder.h"
#include "imu.h"

/*==============================================================================
 [A] CONFIG — Pins, constants, sampling
    - Edit these to match your wiring and desired telemetry rate.
==============================================================================*/
#ifndef LEFT_ENCODER_PIN
#define LEFT_ENCODER_PIN    2      // GP2  : Left wheel encoder input
#endif
#ifndef RIGHT_ENCODER_PIN
#define RIGHT_ENCODER_PIN   6      // GP6  : Right wheel encoder input
#endif
#ifndef PULSES_PER_REV
#define PULSES_PER_REV      20u    // Encoder ticks per one full wheel revolution
#endif
#ifndef KILL_PIN
#define KILL_PIN            21     // GP21 : Kill switch (active LOW to GND)
#endif

#define SAMPLE_MS           500    // Telemetry + drift check interval (milliseconds)

/*==============================================================================
 [B] THRESHOLDS — Drift sensitivity
    - Increase thresholds to make detection less sensitive.
    - Decrease thresholds to flag drift earlier.
==============================================================================*/
#define DRIFT_RPM_THRESHOLD       10.0f  // Flag drift if |L_RPM - R_RPM| > this value
#define DRIFT_HEADING_THRESHOLD    5.0f  // Or if |Δheading(deg)| between samples > this value

/*==============================================================================
 Internal State — Do not modify unless changing polling model
    - Pulse counters updated by polling (via encoder_check_pulse()).
==============================================================================*/
static volatile uint32_t left_pulses  = 0;
static volatile uint32_t right_pulses = 0;

/*==============================================================================
 Helpers — Small inlined utilities for clarity
==============================================================================*/
/// Poll both encoders once and bump counters if a new pulse is observed.
static inline void poll_encoder_pulses(void) {
    if (encoder_check_pulse(LEFT_ENCODER_PIN))  left_pulses++;
    if (encoder_check_pulse(RIGHT_ENCODER_PIN)) right_pulses++;
}

/// Return true while the kill switch is physically pressed (active LOW with pull-up).
static inline bool kill_is_pressed(void) {
    return gpio_get(KILL_PIN) == 0;
}

/*==============================================================================
 main() — System bring-up, control loop, telemetry, drift detection
    EDIT ZONES:
      [C] STARTUP BEHAVIOR  → default speed & motion
      [D] TELEMETRY FORMAT  → printf line; interval is SAMPLE_MS above
      [E] EXTENSIONS        → see "Extension hooks" block
==============================================================================*/
int main(void) {
    // ---- Runtime/USB bring-up (leave as-is) -----------------------------------
    stdio_init_all();
    sleep_ms(300); // allow USB CDC to enumerate cleanly

    // ---- Subsystem initialization (modules own the low-level details) ---------
    motors_init();
    encoder_init(LEFT_ENCODER_PIN);
    encoder_init(RIGHT_ENCODER_PIN);
    imu_init();

    // Kill switch input (internal pull-up so button shorts to GND)
    gpio_init(KILL_PIN);
    gpio_set_dir(KILL_PIN, GPIO_IN);
    gpio_pull_up(KILL_PIN);

    // ---- [C] STARTUP BEHAVIOR --------------------------------------------------
    // Choose your initial behavior: speed (40–100% typical) and motion direction.
    motors_set_speed(40);    // Default duty (%). Change to taste.
    motors_forward();        // Options: motors_forward() / motors_stop() / motors_backward()

    // ---- Telemetry timers and drift state (leave unless you change policy) ----
    absolute_time_t last = get_absolute_time();
    uint32_t last_left  = 0, last_right = 0;
    float    last_heading = 0.0f;
    bool     drift_active = false;

    // One-time boot banner (minimal)
    printf("INIT OK | SPD=%u%% | PPR=%u | KILL=GP%d\n",
           motors_get_speed(), (unsigned)PULSES_PER_REV, KILL_PIN);

    // ================================ MAIN LOOP ================================
    while (true) {

        // ---- Safety: immediate stop & exit on kill switch ---------------------
        if (kill_is_pressed()) {
            motors_stop();
            printf("KILL PRESSED — motors stopped. Exiting.\n");
            fflush(stdout);
            break;
        }

        // ---- High-rate encoder polling (cheap and fast) -----------------------
        poll_encoder_pulses();

        // ---- Periodic work: telemetry + drift check every SAMPLE_MS -----------
        absolute_time_t now = get_absolute_time();
        if (absolute_time_diff_us(last, now) >= (int64_t)SAMPLE_MS * 1000) {

            // Compute RPMs over the last interval
            uint32_t lp = left_pulses  - last_left;
            uint32_t rp = right_pulses - last_right;
            float dt = SAMPLE_MS / 1000.0f;
            float l_rpm = calculate_rpm(lp, PULSES_PER_REV, dt);
            float r_rpm = calculate_rpm(rp, PULSES_PER_REV, dt);

            // Read IMU (yaw/heading, roll, pitch)
            float heading, roll, pitch;
            imu_read(&heading, &roll, &pitch);

            // ---- [D] TELEMETRY FORMAT (single concise line) -------------------
            // Edit this printf if you want different fields or ordering.
            printf("RPM L=%.1f R=%.1f | IMU H=%.1f R=%.1f P=%.1f | SPD=%u%%\n",
                   l_rpm, r_rpm, heading, roll, pitch, motors_get_speed());
            fflush(stdout);

            // ---- Drift detection policy ---------------------------------------
            float rpm_diff     = fabsf(l_rpm - r_rpm);
            float heading_diff = fabsf(heading - last_heading);
            if (heading_diff > 180.0f) heading_diff = 360.0f - heading_diff; // wrap-safe

            bool drift_now = (rpm_diff > DRIFT_RPM_THRESHOLD) ||
                             (heading_diff > DRIFT_HEADING_THRESHOLD);

            if (drift_now && !drift_active) {
                printf("Drift detected: dRPM=%.1f dHead=%.1f°\n", rpm_diff, heading_diff);
                fflush(stdout);
                drift_active = true;
            } else if (!drift_now && drift_active) {
                // Comment out if you prefer no "cleared" message.
                printf("Drift cleared.\n");
                drift_active = false;
            }

            // ---- Commit sample state & advance window --------------------------
            last_left    = left_pulses;
            last_right   = right_pulses;
            last_heading = heading;
            last         = now;
        }

        // Small sleep to reduce CPU load without hurting responsiveness
        sleep_ms(2);
    }
    // ============================== END MAIN LOOP ==============================

    return 0;
}

/*==============================================================================
 [E] EXTENSIONS — Add features here without touching module internals
    Examples:
      - Auto-correct drift:
          * If rpm_diff > X, reduce PWM on the faster wheel by a small step.
          * If heading drifts while commanded straight, bias motor speeds.
      - Logging:
          * Buffer N samples and dump in CSV format every M seconds.
      - Mode control:
          * Add keyboard commands over USB to switch speed/modes with minimal prints.
==============================================================================*/
