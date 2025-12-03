
/*
//With issues
#define PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS 5000

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "motors.h"
#include "encoder.h"
#include "imu.h"
#include "uart_comm.h"
#include "drift_correction.h"

// ===============================
// CONSTANTS
// ===============================
#define LEFT_ENCODER_PIN 2
#define RIGHT_ENCODER_PIN 6

#define WHEEL_RADIUS 0.03f
#define WHEEL_BASE   0.15f
#define PULSES_PER_REV 20

#define MOVE_SPEED_PERCENT 80
#define TURN_SPEED_PERCENT 35
#define TARGET_TURN_ANGLE 90.0f
#define TURN_TIMEOUT_MS 5000
#define SETTLE_TIME_MS 200

// ===============================
// GLOBAL STATE
// ===============================
Pose robot_pose = {0, 0, 0};

static int left_pulses = 0;
static int right_pulses = 0;

// ================================================================
// ODOMETRY UTIL
// ================================================================
static float signed_distance(int pulses) {
    return distance_from_pulses(pulses, PULSES_PER_REV, WHEEL_RADIUS);
}

// ================================================================
// ODOMETRY UPDATE (IMU + ENCODERS)
// ================================================================
static void update_odometry_heading(int lp, int rp) {
    float dL = signed_distance(lp);
    float dR = signed_distance(rp);
    float d  = 0.5f * (dL + dR);

    float heading_deg, roll, pitch;
    imu_read(&heading_deg, &roll, &pitch);

    float heading_rad = heading_deg * (M_PI / 180.0f);

    robot_pose.x += d * cosf(heading_rad);
    robot_pose.y += d * sinf(heading_rad);
    robot_pose.theta = heading_rad;
}

// ================================================================
// TURNING (IMU CONTROL)
// ================================================================
static float angle_diff(float start, float current) {
    float diff = current - start;
    if (diff < -180.0f) diff += 360.0f;
    if (diff > 180.0f) diff -= 360.0f;
    return diff;
}

static void reset_turn_state() {
    motors_stop();
    sleep_ms(50);

    encoder_init(LEFT_ENCODER_PIN);
    encoder_init(RIGHT_ENCODER_PIN);

    float h, r, p;
    for (int i = 0; i < 5; i++) {
        imu_read(&h, &r, &p);
        sleep_ms(10);
    }

    sleep_ms(SETTLE_TIME_MS);
}

static void turn_left() {
    float start, r, p;
    imu_read(&start, &r, &p);

    motors_left();
    uint32_t t0 = to_ms_since_boot(get_absolute_time());

    while (true) {
        float now;
        imu_read(&now, &r, &p);

        if (fabsf(angle_diff(start, now)) >= TARGET_TURN_ANGLE) break;
        if (to_ms_since_boot(get_absolute_time()) - t0 > TURN_TIMEOUT_MS) break;
    }

    reset_turn_state();
}

static void turn_right() {
    float start, r, p;
    imu_read(&start, &r, &p);

    motors_right();
    uint32_t t0 = to_ms_since_boot(get_absolute_time());

    while (true) {
        float now;
        imu_read(&now, &r, &p);

        if (fabsf(angle_diff(start, now)) >= TARGET_TURN_ANGLE) break;
        if (to_ms_since_boot(get_absolute_time()) - t0 > TURN_TIMEOUT_MS) break;
    }

    reset_turn_state();
}

// ================================================================
// MAIN
// ================================================================
int main() {
    stdio_init_all();
    sleep_ms(500);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    for (int i = 0; i < 3; i++) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1); sleep_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, 0); sleep_ms(100);
    }

    printf("\n=== IMU + ODOMETRY CONTROLLED ROBOT ===\n");

    imu_init();
    drift_correction_init();

    // REINIT ENCODERS after drift correction
    encoder_init(LEFT_ENCODER_PIN);
    encoder_init(RIGHT_ENCODER_PIN);

    uart_comm_init();
    printf("Waiting for commands...\n");

    // ============================================================
    // AUTO-TEST: Run forward for 2 seconds at startup
    // ============================================================
    printf("\n=== AUTO TEST: FORWARD 2 SECONDS ===\n");

    motors_set_speed(40);
    motors_forward();

    uint32_t start = to_ms_since_boot(get_absolute_time());

    while (to_ms_since_boot(get_absolute_time()) - start < 2000) {

        if (encoder_check_pulse(LEFT_ENCODER_PIN))  left_pulses++;
        if (encoder_check_pulse(RIGHT_ENCODER_PIN)) right_pulses++;

        update_odometry_heading(left_pulses, right_pulses);

        printf("LP=%d  RP=%d  X=%.3f  Y=%.3f\n",
            left_pulses, right_pulses,
            robot_pose.x, robot_pose.y);

        left_pulses = 0;
        right_pulses = 0;

        sleep_ms(10);
    }

    motors_stop();
    printf("=== AUTO TEST COMPLETE ===\n\n");

    // ============================================================
    // NORMAL RUNTIME LOOP (COMMANDS FROM LIDAR)
    // ============================================================
    char msg_buffer[UART_MAX_MESSAGE_LEN];
    char active_command = 0;

    while (true) {

        // Read encoder pulses
        if (encoder_check_pulse(LEFT_ENCODER_PIN))  left_pulses++;
        if (encoder_check_pulse(RIGHT_ENCODER_PIN)) right_pulses++;

        uart_comm_process();

        if (uart_receive_message(msg_buffer, UART_MAX_MESSAGE_LEN)) {
            if (strlen(msg_buffer) > 0)
                active_command = msg_buffer[0];
        }

        switch (active_command) {

            case 'w': case 'W':
                motors_set_speed(MOVE_SPEED_PERCENT);
                motors_forward();
                update_odometry_heading(left_pulses, right_pulses);
                drift_correction_update();
                left_pulses = 0;
                right_pulses = 0;
                break;

            case 's': case 'S':
                motors_set_speed(MOVE_SPEED_PERCENT);
                motors_backward();
                update_odometry_heading(-left_pulses, -right_pulses);
                left_pulses = 0;
                right_pulses = 0;
                break;

            case 'a': case 'A':
                turn_right();
                uart_send_string("done");
                active_command = 0;
                break;

            case 'd': case 'D':
                turn_left();
                uart_send_string("done");
                active_command = 0;
                break;

            case 'q': case 'Q':
                motors_stop();
                active_command = 0;
                break;

            default:
                // motors_stop();
                // break;
                motors_set_speed(MOVE_SPEED_PERCENT);
                motors_forward();
                update_odometry_heading(left_pulses, right_pulses);
                drift_correction_update();
                left_pulses = 0;
                right_pulses = 0;
                break;
        }

        printf("POS => X: %.3f  Y: %.3f  THETA: %.2f°\n",
            robot_pose.x,
            robot_pose.y,
            robot_pose.theta * (180.0f / M_PI));

        sleep_ms(10);
    }

    return 0;
}

*/
/*


//original version
#define PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS 5000

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "motors.h"
#include "encoder.h"
#include "imu.h"
#include "uart_comm.h"
#include "drift_correction.h"

// === SETTINGS ===
#define TURN_SPEED_PERCENT 35
#define MOVE_SPEED_PERCENT 80
#define TURN_TIMEOUT_MS 5000
#define SETTLE_TIME_MS 200

#define LEFT_ENCODER_PIN 2
#define RIGHT_ENCODER_PIN 6

#define TARGET_TURN_ANGLE 90.0f

// =========================
// ANGLE DIFF
// =========================
static float get_angle_diff(float start, float current) {
    float diff = current - start;
    if (diff < -180.0f) diff += 360.0f;
    if (diff > 180.0f) diff -= 360.0f;
    return diff;
}

// =========================
// RESET TURN STATE
// =========================
static void reset_turn_state() {
    motors_stop();
    sleep_ms(50);

    encoder_init(LEFT_ENCODER_PIN);
    encoder_init(RIGHT_ENCODER_PIN);

    float h, r, p;
    for (int i = 0; i < 5; i++) {
        imu_read(&h, &r, &p);
        sleep_ms(30);
    }

    sleep_ms(SETTLE_TIME_MS);
}

// =========================
// TURN LEFT
// =========================
static bool execute_left_turn() {
    printf("\n=== LEFT TURN (IMU CONTROL) ===\n");

    float start_heading, roll, pitch;
    imu_read(&start_heading, &roll, &pitch);

    motors_left();

    int pulse_count = 0;
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    float current_heading = start_heading;
    float turned_so_far = 0.0f;

    printf("Start Turn\n");

    while (turned_so_far < TARGET_TURN_ANGLE) {
        imu_read(&current_heading, &roll, &pitch);
        float diff = get_angle_diff(start_heading, current_heading);
        turned_so_far = fabsf(diff);

        if (encoder_check_pulse(LEFT_ENCODER_PIN)) pulse_count++;

        if (to_ms_since_boot(get_absolute_time()) - start_time > TURN_TIMEOUT_MS) {
            motors_stop();
            return false;
        }
    }

    motors_stop();
    reset_turn_state();
    return true;
}

// =========================
// TURN RIGHT
// =========================
static bool execute_right_turn() {
    printf("\n=== RIGHT TURN (IMU CONTROL) ===\n");

    float start_heading, roll, pitch;
    imu_read(&start_heading, &roll, &pitch);

    motors_right();

    int pulse_count = 0;
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    float current_heading = start_heading;
    float turned_so_far = 0.0f;

    printf("Start Turn\n");

    while (turned_so_far < TARGET_TURN_ANGLE) {
        imu_read(&current_heading, &roll, &pitch);
        float diff = get_angle_diff(start_heading, current_heading);
        turned_so_far = fabsf(diff);

        if (encoder_check_pulse(RIGHT_ENCODER_PIN)) pulse_count++;

        if (to_ms_since_boot(get_absolute_time()) - start_time > TURN_TIMEOUT_MS) {
            motors_stop();
            return false;
        }
    }

    motors_stop();
    reset_turn_state();
    return true;
}

// =========================
// FORWARD — DRIFT CORRECTED
// =========================
static bool execute_forward() {
    printf("\n=== FORWARD (CONTINUOUS, DRIFT-CORRECTED) ===\n");

    motors_forward();

    motors_forward_bias(0, 0);
    drift_correction_start();

    drift_correction_update();
    sleep_ms(1);

    return true;
}

// =========================
// BACKWARD
// =========================
static bool execute_backward() {
    printf("\n=== BACKWARD (CONTINUOUS MOVEMENT) ===\n");

    motors_backward();

    if (encoder_check_pulse(LEFT_ENCODER_PIN)) {}
    if (encoder_check_pulse(RIGHT_ENCODER_PIN)) {}

    sleep_ms(10);
    return true;
}

// =========================
// MAIN
// =========================
int main() {
    stdio_init_all();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    for (int i = 0; i < 3; i++) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(100);
    }

    printf("\n=== IMU CONTROLLED ROBOT ===\n");

    drift_correction_init();
    imu_init();
    uart_comm_init();

    sleep_ms(1000);

    printf("Waiting for commands...\n");

    char msg_buffer[UART_MAX_MESSAGE_LEN];

    while (true) {
        printf("\n[DEBUG] --- Start Loop Iteration ---\n");

        uart_comm_process();
        printf("[DEBUG] UART communication process completed.\n");

        char command = 0;

        while (uart_receive_message(msg_buffer, UART_MAX_MESSAGE_LEN)) {
            printf("[DEBUG] Message received: '%s'\n", msg_buffer);
            if (strlen(msg_buffer) > 0) {
                command = msg_buffer[0];
                printf("[DEBUG] Command extracted: '%c'\n", command);
            }
        }

        if (command == 0) {
            printf("[DEBUG] No new command received.\n");
        } else {
            printf("[DEBUG] Processing command '%c'\n", command);
        }

        switch (command) {
            case 'w':
            case 'W':
                execute_forward();
                break;

            case 's':
            case 'S':
                execute_backward();
                break;

            case 'a':
            case 'A':
                execute_right_turn();
                uart_send_string("done");
                sleep_ms(1000);
                break;

            case 'd':
            case 'D':
                execute_left_turn();
                uart_send_string("done");
                sleep_ms(1000);
                break;

            case 'h':
            case 'H': {
                float h, r, p;
                imu_read(&h, &r, &p);
                printf("Heading: %.1f\n", h);
                break;
            }

            case 'q':
            case 'Q':
                motors_stop();
                printf("Shutting down...\n");
                break;

            default:
                printf("No Commands!\n");
                execute_forward();
                break;
        }

        printf("[DEBUG] Sleeping 10ms\n");
        sleep_ms(10);
    }

    return 0;
}*/



//Updated version with odometry heading update after turns
#define PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS 5000

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "motors.h"
#include "encoder.h"
#include "imu.h"
#include "uart_comm.h"
#include "drift_correction.h"

// ===============================
// CONSTANTS
// ===============================
#define LEFT_ENCODER_PIN 2
#define RIGHT_ENCODER_PIN 6

#define WHEEL_RADIUS 0.03f
#define WHEEL_BASE   0.15f
#define PULSES_PER_REV 20

#define MOVE_SPEED_PERCENT 80
#define TURN_SPEED_PERCENT 35
#define TARGET_TURN_ANGLE 90.0f
#define TURN_TIMEOUT_MS 5000
#define SETTLE_TIME_MS 200

// ===============================
// GLOBAL STATE
// ===============================
Pose robot_pose = {0, 0, 0};

static int left_pulses = 0;
static int right_pulses = 0;

// ================================================================
// ODOMETRY UTIL
// ================================================================
static float signed_distance(int pulses) {
    return distance_from_pulses(pulses, PULSES_PER_REV, WHEEL_RADIUS);
}

// ================================================================
// ODOMETRY UPDATE (IMU + ENCODERS)
// ================================================================
static void update_odometry_heading(int lp, int rp) {
    float dL = signed_distance(lp);
    float dR = signed_distance(rp);
    float d  = 0.5f * (dL + dR);

    float heading_deg, roll, pitch;
    imu_read(&heading_deg, &roll, &pitch);

    float heading_rad = heading_deg * (M_PI / 180.0f);

    robot_pose.x += d * cosf(heading_rad);
    robot_pose.y += d * sinf(heading_rad);
    robot_pose.theta = heading_rad;
}

// ================================================================
// TURNING (IMU CONTROL)
// ================================================================
static float angle_diff(float start, float current) {
    float diff = current - start;
    if (diff < -180.0f) diff += 360.0f;
    if (diff > 180.0f) diff -= 360.0f;
    return diff;
}

static void reset_turn_state() {
    motors_stop();
    sleep_ms(50);

    encoder_init(LEFT_ENCODER_PIN);
    encoder_init(RIGHT_ENCODER_PIN);

    float h, r, p;
    for (int i = 0; i < 5; i++) {
        imu_read(&h, &r, &p);
        sleep_ms(10);
    }

    sleep_ms(SETTLE_TIME_MS);
}

// ================================================================
// FIXED TURN LEFT (UPDATES POSE THETA)
// ================================================================
static void turn_left() {
    float start, r, p;
    imu_read(&start, &r, &p);

    motors_left();
    uint32_t t0 = to_ms_since_boot(get_absolute_time());

    while (true) {
        float now;
        imu_read(&now, &r, &p);

        if (fabsf(angle_diff(start, now)) >= TARGET_TURN_ANGLE) break;
        if (to_ms_since_boot(get_absolute_time()) - t0 > TURN_TIMEOUT_MS) break;
    }

    reset_turn_state();

    // === FIX: UPDATE POSE θ AFTER TURN ===
    float h;
    imu_read(&h, &r, &p);
    robot_pose.theta = h * (M_PI / 180.0f);
}

// ================================================================
// FIXED TURN RIGHT (UPDATES POSE THETA)
// ================================================================
static void turn_right() {
    float start, r, p;
    imu_read(&start, &r, &p);

    motors_right();
    uint32_t t0 = to_ms_since_boot(get_absolute_time());

    while (true) {
        float now;
        imu_read(&now, &r, &p);

        if (fabsf(angle_diff(start, now)) >= TARGET_TURN_ANGLE) break;
        if (to_ms_since_boot(get_absolute_time()) - t0 > TURN_TIMEOUT_MS) break;
    }

    reset_turn_state();

    // === FIX: UPDATE POSE θ AFTER TURN ===
    float h;
    imu_read(&h, &r, &p);
    robot_pose.theta = h * (M_PI / 180.0f);
}

// ================================================================
// MAIN
// ================================================================
int main() {
    stdio_init_all();
    sleep_ms(500);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    for (int i = 0; i < 3; i++) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1); sleep_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, 0); sleep_ms(100);
    }

    printf("\n=== IMU + ODOMETRY CONTROLLED ROBOT ===\n");

    imu_init();
    drift_correction_init();

    // REINIT ENCODERS after drift correction
    encoder_init(LEFT_ENCODER_PIN);
    encoder_init(RIGHT_ENCODER_PIN);

    uart_comm_init();
    printf("Waiting for commands...\n");

    // ============================================================
    // AUTO-TEST: Run forward for 2 seconds at startup
    // ============================================================
    printf("\n=== AUTO TEST: FORWARD 2 SECONDS ===\n");

    motors_set_speed(40);
    motors_forward();

    uint32_t start = to_ms_since_boot(get_absolute_time());

    while (to_ms_since_boot(get_absolute_time()) - start < 2000) {

        if (encoder_check_pulse(LEFT_ENCODER_PIN))  left_pulses++;
        if (encoder_check_pulse(RIGHT_ENCODER_PIN)) right_pulses++;

        update_odometry_heading(left_pulses, right_pulses);

        printf("LP=%d  RP=%d  X=%.3f  Y=%.3f\n",
            left_pulses, right_pulses,
            robot_pose.x, robot_pose.y);

        left_pulses = 0;
        right_pulses = 0;

        sleep_ms(10);
    }

    motors_stop();
    printf("=== AUTO TEST COMPLETE ===\n\n");

    // ============================================================
    // NORMAL RUNTIME LOOP (COMMANDS FROM LIDAR)
    // ============================================================
    char msg_buffer[UART_MAX_MESSAGE_LEN];
    char active_command = 0;

    while (true) {

        // Read encoder pulses
        if (encoder_check_pulse(LEFT_ENCODER_PIN))  left_pulses++;
        if (encoder_check_pulse(RIGHT_ENCODER_PIN)) right_pulses++;

        uart_comm_process();

        if (uart_receive_message(msg_buffer, UART_MAX_MESSAGE_LEN)) {
            if (strlen(msg_buffer) > 0)
                active_command = msg_buffer[0];
        }

        switch (active_command) {

            case 'w': case 'W':
                motors_set_speed(MOVE_SPEED_PERCENT);
                motors_forward();
                motors_forward_bias(0, 0);
                update_odometry_heading(left_pulses, right_pulses);
                drift_correction_update();
                left_pulses = 0;
                right_pulses = 0;
                break;

            case 's': case 'S':
                motors_set_speed(MOVE_SPEED_PERCENT);
                motors_backward();
                update_odometry_heading(-left_pulses, -right_pulses);
                left_pulses = 0;
                right_pulses = 0;
                break;

            case 'a': case 'A':
                turn_right();
                uart_send_string("done");
                active_command = 0;
                break;

            case 'd': case 'D':
                turn_left();
                uart_send_string("done");
                active_command = 0;
                break;

            case 'q': case 'Q':
                motors_stop();
                active_command = 0;
                break;

            default:
                motors_set_speed(MOVE_SPEED_PERCENT);
                motors_forward();
                motors_forward_bias(0, 0);
                update_odometry_heading(left_pulses, right_pulses);
                drift_correction_update();
                sleep_ms(1);
                left_pulses = 0;
                right_pulses = 0;
                break;
        }

        printf("POS => X: %.3f  Y: %.3f  THETA: %.2f°\n",
            robot_pose.x,
            robot_pose.y,
            robot_pose.theta * (180.0f / M_PI));

        sleep_ms(10);
    }

    return 0;
}
