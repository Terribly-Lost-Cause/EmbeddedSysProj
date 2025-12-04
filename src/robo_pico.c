

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

#define MOVE_SPEED_PERCENT 50
#define TURN_SPEED_PERCENT 70
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

    motors_left(); // Start turning fast
    uint32_t t0 = to_ms_since_boot(get_absolute_time());

    while (true) {
        float now;
        imu_read(&now, &r, &p);
        float diff = fabsf(angle_diff(start, now));

        // 1. Check if we are done
        if (diff >= TARGET_TURN_ANGLE) break;

        // 2. SLOW DOWN if we are close (within 20 degrees)
        if (TARGET_TURN_ANGLE - diff < 20.0f) {
            motors_set_speed(40); // Slow down to 40% speed
            motors_left();        // Re-apply slow speed
        }

        // Timeout safety
        if (to_ms_since_boot(get_absolute_time()) - t0 > TURN_TIMEOUT_MS) break;
    }

    reset_turn_state();
    motors_set_speed(MOVE_SPEED_PERCENT); // Restore normal speed

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

    motors_right(); // Start turning fast
    uint32_t t0 = to_ms_since_boot(get_absolute_time());

    while (true) {
        float now;
        imu_read(&now, &r, &p);
        float diff = fabsf(angle_diff(start, now));

        // 1. Check if we are done
        if (diff >= TARGET_TURN_ANGLE) break;

        // 2. SLOW DOWN if we are close (within 20 degrees)
        if (TARGET_TURN_ANGLE - diff < 20.0f) {
            motors_set_speed(40); // Slow down to 40% speed
            motors_right();       // Re-apply slow speed (RIGHT)
        }

        // Timeout safety
        if (to_ms_since_boot(get_absolute_time()) - t0 > TURN_TIMEOUT_MS) break;
    }

    reset_turn_state();
    motors_set_speed(MOVE_SPEED_PERCENT); // Restore normal speed

    // === FIX: UPDATE POSE θ AFTER TURN ===
    float h;
    imu_read(&h, &r, &p);
    robot_pose.theta = h * (M_PI / 180.0f);
}

int telemetry_counter = 0;
char status_str[] = "none";
bool is_moving_forward = false;

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
    char tele_buffer[UART_MAX_MESSAGE_LEN];
    char active_command = 0;
    bool is_moving_forward = false;

    while (true) {

        // Read encoder pulses
        if (encoder_check_pulse(LEFT_ENCODER_PIN))  left_pulses++;
        if (encoder_check_pulse(RIGHT_ENCODER_PIN)) right_pulses++;

        uart_comm_process();
        

        if (uart_receive_message(msg_buffer, UART_MAX_MESSAGE_LEN)) {
            if (strlen(msg_buffer) > 0)
                active_command = msg_buffer[0];
        }

        telemetry_counter++;
        if (telemetry_counter >= 10) { // Send only every 10th time (approx 100ms)
            sprintf(tele_buffer, "PX,%.1f,%.1f,%.2f,%s", 
                robot_pose.x, 
                robot_pose.y, 
                robot_pose.theta * (180.0f / M_PI), 
                status_str
            );
            uart_send_tele(tele_buffer);
            telemetry_counter = 0;
        }

        

        switch (active_command) {
            // printf("Active Command: '%c' \n", active_command); // Optional: Comment out to reduce spam

            case 'w': case 'W':
                strcpy(status_str, "none");
                if (!is_moving_forward) {
                    drift_correction_start();
                    is_moving_forward = true;
                }
                
                motors_set_speed(MOVE_SPEED_PERCENT);
                motors_forward();
                                
                update_odometry_heading(left_pulses, right_pulses);
                drift_correction_update();
                // printf("Move Forward");

                left_pulses = 0;
                right_pulses = 0;
                break;

            case 's': case 'S':
                if (is_moving_forward) {
                    drift_correction_stop();
                    is_moving_forward = false;
                }
                
                motors_set_speed(MOVE_SPEED_PERCENT);
                motors_backward();
                update_odometry_heading(-left_pulses, -right_pulses);
                printf("Backward\n");
                sleep_ms(2000);

                strcpy(status_str, "done");
                sprintf(tele_buffer, "PX,%.1f,%.1f,%.2f,%s", 
                    robot_pose.x, 
                    robot_pose.y, 
                    robot_pose.theta * (180.0f / M_PI), 
                    status_str
                );
                uart_send_tele(tele_buffer);

                left_pulses = 0;
                right_pulses = 0;
                break;

            case 'd': case 'D':
                if (is_moving_forward) {
                    drift_correction_stop();
                    is_moving_forward = false;
                }
                turn_right();
                printf("Right\n"); // Fixed label (was Left)
                sleep_ms(2000);

                strcpy(status_str, "done");

                sprintf(tele_buffer, "PX,%.1f,%.1f,%.2f,%s", 
                    robot_pose.x, 
                    robot_pose.y, 
                    robot_pose.theta * (180.0f / M_PI), 
                    status_str
                );
                uart_send_tele(tele_buffer);

                
                active_command = 0;
                break;

            case 'a': case 'A':
                if (is_moving_forward) {
                    drift_correction_stop();
                    is_moving_forward = false;
                }
                turn_left();
                strcpy(status_str, "done");
                printf("Left\n"); // Fixed label (was Right)
                sleep_ms(2000);

                sprintf(tele_buffer, "PX,%.1f,%.1f,%.2f,%s", 
                    robot_pose.x, 
                    robot_pose.y, 
                    robot_pose.theta * (180.0f / M_PI), 
                    status_str
                );
                uart_send_tele(tele_buffer);

                
                active_command = 0;
                break;

            case 'q': case 'Q':

                strcpy(status_str, "none");

                if (is_moving_forward) {
                    drift_correction_stop();
                    is_moving_forward = false;
                }
                motors_stop();
                active_command = 0;
                break;

            default:
                // If we fall here, we should probably stop everything to be safe
                if (is_moving_forward) {
                    drift_correction_stop();
                    is_moving_forward = false;
                }
                motors_stop(); // Ensure motors are stopped in default state
                
                update_odometry_heading(left_pulses, right_pulses);
                sleep_ms(1);
                left_pulses = 0;
                right_pulses = 0;
                break;
        }

        // printf("POS => X: %.3f  Y: %.3f  THETA: %.2f°\n",
        //     robot_pose.x,
        //     robot_pose.y,
        //     robot_pose.theta * (180.0f / M_PI));

        sleep_us(100);
    }

    return 0;
}