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
#define MOVE_SPEED_PERCENT 30
#define TURN_TIMEOUT_MS 5000
#define SETTLE_TIME_MS 200

#define LEFT_ENCODER_PIN 2
#define RIGHT_ENCODER_PIN 6  

#define TARGET_TURN_ANGLE 90.0f

static float get_angle_diff(float start, float current) {
    float diff = current - start;
    if (diff < -180.0f) diff += 360.0f;
    if (diff > 180.0f) diff -= 360.0f;
    return diff;
}

static void reset_turn_state() {
    motors_stop();
    sleep_ms(50);
    motors_set_speed(TURN_SPEED_PERCENT);

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
// TURNING (IMU CONTROL)
// =========================
static bool execute_left_turn() {
    printf("\n=== LEFT TURN (IMU CONTROL) ===\n");

    float start_heading, roll, pitch;
    imu_read(&start_heading, &roll, &pitch);

    motors_set_speed(TURN_SPEED_PERCENT);
    motors_left();

    int pulse_count = 0;
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    float current_heading = start_heading;
    float turned_so_far = 0.0f;

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

static bool execute_right_turn() {
    printf("\n=== RIGHT TURN (IMU CONTROL) ===\n");

    float start_heading, roll, pitch;
    imu_read(&start_heading, &roll, &pitch);

    motors_set_speed(TURN_SPEED_PERCENT);
    motors_right();

    int pulse_count = 0;
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    float current_heading = start_heading;
    float turned_so_far = 0.0f;

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
// FORWARD — CONTINUOUS
// DRIFT-CORRECTED
// =========================
static bool execute_forward() {
    printf("\n=== FORWARD (CONTINUOUS, DRIFT-CORRECTED) ===\n");

    drift_correction_start();
    motors_set_speed(MOVE_SPEED_PERCENT);
    motors_forward();

    uint32_t left_count = 0;
    uint32_t right_count = 0;

    while (true) {

        // Optional: pulse counts for debugging
        if (encoder_check_pulse(LEFT_ENCODER_PIN))  left_count++;
        if (encoder_check_pulse(RIGHT_ENCODER_PIN)) right_count++;

        // Keep wheels matched
        drift_correction_update();

        // Read new user commands
        int ch = getchar_timeout_us(0);

        if (ch == 's' || ch == 'S' ||   // backward
            ch == 'a' || ch == 'A' ||   // turn right
            ch == 'd' || ch == 'D' ||   // turn left
            ch == 'q' || ch == 'Q') {   // quit

            printf("Stopping forward because of command.\n");
            motors_stop();
            drift_correction_stop();
            break;
        }

        // IMPORTANT:
        // 'w' or 'W' should NOT stop forward drive.
        // That means: forward continues if user keeps sending 'w'.

        sleep_ms(10);
    }

    return true;
}

// =========================
// BACKWARD (NO DRIFT CORR)
// =========================
static bool execute_backward() {
    printf("\n=== BACKWARD (CONTINUOUS MOVEMENT) ===\n");

    motors_set_speed(MOVE_SPEED_PERCENT);
    motors_backward();

    uint32_t left_count = 0;
    uint32_t right_count = 0;

    while (true) {

        // Optional debug pulses
        if (encoder_check_pulse(LEFT_ENCODER_PIN))  left_count++;
        if (encoder_check_pulse(RIGHT_ENCODER_PIN)) right_count++;

        // Stop backward ONLY when a *different* command arrives
        int ch = getchar_timeout_us(0);

        if (ch == 'w' || ch == 'W' ||   // forward
            ch == 'a' || ch == 'A' ||   // right turn
            ch == 'd' || ch == 'D' ||   // left turn
            ch == 'q' || ch == 'Q') {   // quit

            printf("Stopping backward because of command.\n");
            motors_stop();
            break;
        }

        // IMPORTANT: s/S SHOULD NOT STOP BACKWARD
        // If ch == 's', we simply ignore it and continue moving backward.

        sleep_ms(10);
    }

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
        gpio_put(PICO_DEFAULT_LED_PIN, 1); sleep_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, 0); sleep_ms(100);
    }

    printf("\n=== IMU CONTROLLED ROBOT ===\n");

    drift_correction_init();   // motors + encoders
    imu_init();

    uart_comm_init();
    printf("Waiting for commands...\n");

    char msg_buffer[UART_MAX_MESSAGE_LEN];

    while (true) {
        uart_comm_process();

        char command = 0;
        bool from_uart = false;

        if (uart_receive_message(msg_buffer, UART_MAX_MESSAGE_LEN)) {
            if (strlen(msg_buffer) > 0) {
                command = msg_buffer[0];
                from_uart = true;
            }
        }

        int ch = getchar_timeout_us(0);
        if (ch != PICO_ERROR_TIMEOUT && !command)
            command = (char)ch;

        if (command) {

            if (command == 'w' || command == 'W') execute_forward();
            else if (command == 's' || command == 'S') execute_backward();
            else if (command == 'a' || command == 'A') execute_right_turn();
            else if (command == 'd' || command == 'D') execute_left_turn();
            else if (command == 'h' || command == 'H') {
                float h, r, p;
                imu_read(&h, &r, &p);
                printf("Heading: %.1f\n", h);
            }
            else if (command == 'q' || command == 'Q') {
                motors_stop();
                printf("Shutting down...\n");
                break;
            }

            if (from_uart &&
               (command=='w'||command=='W'||command=='s'||command=='S'||
                command=='a'||command=='A'||command=='d'||command=='D')) {
                uart_send_string("done");
            }
        }

        sleep_ms(10);
    }

    return 0;
}
