#define PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS 5000

#include <stdio.h>
#include <string.h>
#include <math.h> 
#include "pico/stdlib.h"
#include "motors.h"
#include "encoder.h"
#include "imu.h"
#include "uart_comm.h"

// === SETTINGS ===
#define TURN_SPEED_PERCENT 35
#define MOVE_SPEED_PERCENT 30
#define TURN_TIMEOUT_MS 5000
#define SETTLE_TIME_MS 200
#define LEFT_ENCODER_PIN 2

#define TARGET_TURN_ANGLE 90.0f
#define FORWARD_DISTANCE_PULSES 40  // Adjust based on your encoder (roughly 2 wheel rotations)
#define BACKWARD_DISTANCE_PULSES 20 // Roughly 1 wheel rotation for 20cm backup

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
    
    // Reset IMU readings to prevent drift accumulation
    float h, r, p;
    for (int i = 0; i < 5; i++) {
        imu_read(&h, &r, &p);
        sleep_ms(30);
    }
    
    sleep_ms(SETTLE_TIME_MS);
}

static bool execute_left_turn() {
    printf("\n╔═══════════════════════════════════════╗\n");
    printf("║  EXECUTING LEFT TURN (IMU CONTROL)     ║\n");
    printf("╚═══════════════════════════════════════╝\n");
    
    float start_heading, roll, pitch;
    imu_read(&start_heading, &roll, &pitch);
    printf("  Start heading: %.1f°\n", start_heading);
    printf("  Target Turn:   %.1f°\n", TARGET_TURN_ANGLE);
    
    motors_set_speed(TURN_SPEED_PERCENT);
    motors_left();
    
    int pulse_count = 0;
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    float current_heading = start_heading;
    float turned_so_far = 0.0f;

    printf("  Turning...\n");
    
    while (turned_so_far < TARGET_TURN_ANGLE) {
        imu_read(&current_heading, &roll, &pitch);
        
        float diff = get_angle_diff(start_heading, current_heading);
        turned_so_far = fabsf(diff);
        
        if (encoder_check_pulse(LEFT_ENCODER_PIN)) {
            pulse_count++;
            printf("    Pulse: %2d | Heading: %6.1f° | Turned: %5.1f° / 90.0°\n", 
                   pulse_count, current_heading, turned_so_far);
        }
        
        if (to_ms_since_boot(get_absolute_time()) - start_time > TURN_TIMEOUT_MS) {
            printf("\n  ⚠️  TIMEOUT! Sensor may be stuck.\n");
            motors_stop();
            return false;
        }
    }
    
    motors_stop();
    
    printf("\n  ✓ Target Reached!\n");
    printf("  Final Heading: %.1f°\n", current_heading);
    printf("  Total Turned:  %.1f°\n", turned_so_far);
    
    reset_turn_state();
    return true;
}

static bool execute_right_turn() {
    printf("\n╔═══════════════════════════════════════╗\n");
    printf("║  EXECUTING RIGHT TURN (IMU CONTROL)    ║\n");
    printf("╚═══════════════════════════════════════╝\n");
    
    float start_heading, roll, pitch;
    imu_read(&start_heading, &roll, &pitch);
    printf("  Start heading: %.1f°\n", start_heading);
    printf("  Target Turn:   %.1f°\n", TARGET_TURN_ANGLE);
    
    motors_set_speed(TURN_SPEED_PERCENT);
    motors_right();
    
    int pulse_count = 0;
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    float current_heading = start_heading;
    float turned_so_far = 0.0f;

    printf("  Turning...\n");
    
    while (turned_so_far < TARGET_TURN_ANGLE) {
        imu_read(&current_heading, &roll, &pitch);
        
        float diff = get_angle_diff(start_heading, current_heading);
        turned_so_far = fabsf(diff);
        
        if (encoder_check_pulse(LEFT_ENCODER_PIN)) {
            pulse_count++;
            printf("    Pulse: %2d | Heading: %6.1f° | Turned: %5.1f° / 90.0°\n", 
                   pulse_count, current_heading, turned_so_far);
        }
        
        if (to_ms_since_boot(get_absolute_time()) - start_time > TURN_TIMEOUT_MS) {
            printf("\n  ⚠️  TIMEOUT! Sensor may be stuck.\n");
            motors_stop();
            return false;
        }
    }
    
    motors_stop();
    
    printf("\n  ✓ Target Reached!\n");
    printf("  Final Heading: %.1f°\n", current_heading);
    printf("  Total Turned:  %.1f°\n", turned_so_far);
    
    reset_turn_state();
    return true;
}

static bool execute_forward() {
    printf("\n╔═══════════════════════════════════════╗\n");
    printf("║  EXECUTING FORWARD MOVEMENT            ║\n");
    printf("╚═══════════════════════════════════════╝\n");
    
    motors_set_speed(MOVE_SPEED_PERCENT);
    motors_forward();
    
    uint32_t pulse_count = 0;
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    
    printf("  Moving forward...\n");
    
    while (pulse_count < FORWARD_DISTANCE_PULSES) {
        if (encoder_check_pulse(LEFT_ENCODER_PIN)) {
            pulse_count++;
            printf("    Pulse: %2lu / %d\n", pulse_count, FORWARD_DISTANCE_PULSES);
        }
        
        if (to_ms_since_boot(get_absolute_time()) - start_time > 10000) {
            printf("\n  ⚠️  TIMEOUT!\n");
            motors_stop();
            return false;
        }
    }
    
    motors_stop();
    printf("\n  ✓ Forward movement complete!\n");
    
    sleep_ms(SETTLE_TIME_MS);
    return true;
}

static bool execute_backward() {
    printf("\n╔═══════════════════════════════════════╗\n");
    printf("║  EXECUTING BACKWARD MOVEMENT           ║\n");
    printf("╚═══════════════════════════════════════╝\n");
    
    motors_set_speed(MOVE_SPEED_PERCENT);
    motors_backward();
    
    uint32_t pulse_count = 0;
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    
    printf("  Moving backward...\n");
    
    while (pulse_count < BACKWARD_DISTANCE_PULSES) {
        if (encoder_check_pulse(LEFT_ENCODER_PIN)) {
            pulse_count++;
            printf("    Pulse: %2lu / %d\n", pulse_count, BACKWARD_DISTANCE_PULSES);
        }
        
        if (to_ms_since_boot(get_absolute_time()) - start_time > 10000) {
            printf("\n  ⚠️  TIMEOUT!\n");
            motors_stop();
            return false;
        }
    }
    
    motors_stop();
    printf("\n  ✓ Backward movement complete!\n");
    
    sleep_ms(SETTLE_TIME_MS);
    return true;
}

int main() {
    stdio_init_all();
    
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    for (int i = 0; i < 3; i++) {
        gpio_put(LED_PIN, 1); sleep_ms(100);
        gpio_put(LED_PIN, 0); sleep_ms(100);
    }
    
    printf("\n\n=== IMU CONTROLLED ROBOT ===\n");
    printf("Using HARDCODED Calibration.\n");
    printf("Move Speed: %d%%, Turn Speed: %d%%\n", MOVE_SPEED_PERCENT, TURN_SPEED_PERCENT);
    printf("Commands (UART or Keyboard):\n");
    printf("  'w' - Move Forward\n");
    printf("  's' - Move Backward\n");
    printf("  'a' - Turn Left 90 Deg (IMU Auto-Stop)\n");
    printf("  'd' - Turn Right 90 Deg (IMU Auto-Stop)\n");
    printf("  'h' - Show Heading\n");
    printf("  'q' - Quit\n");
    
    motors_init();
    motors_set_speed(TURN_SPEED_PERCENT);
    
    imu_init();
    printf("IMU Initialized.\n");
    
    encoder_init(LEFT_ENCODER_PIN);
    
    // Initialize UART communication
    uart_comm_init();
    printf("UART Communication Initialized.\n");
    printf("Waiting for commands...\n");
    
    char msg_buffer[UART_MAX_MESSAGE_LEN];
    
    while (true) {
        // Process incoming UART messages
        uart_comm_process();
        
        char command = 0;
        bool from_uart = false;
        
        // Check for UART command
        if (uart_receive_message(msg_buffer, UART_MAX_MESSAGE_LEN)) {
            if (strlen(msg_buffer) > 0) {
                command = msg_buffer[0];
                from_uart = true;
                printf("\n>>> UART Command Received: %c <<<\n", command);
            }
        }
        
        // Check for keyboard input (for manual testing)
        int ch = getchar_timeout_us(0);
        if (ch != PICO_ERROR_TIMEOUT && command == 0) {
            command = (char)ch;
            from_uart = false;
        }
        
        // Process command
        if (command != 0) {
            bool success = false;
            
            if (command == 'w' || command == 'W') {
                success = execute_forward();
            }
            else if (command == 's' || command == 'S') {
                success = execute_backward();
            }
            else if (command == 'a' || command == 'A') {
                success = execute_right_turn();  // 'a' physically turns left (calls motors_right)
            }
            else if (command == 'd' || command == 'D') {
                success = execute_left_turn();   // 'd' physically turns right (calls motors_left)
            }
            else if (command == 'h' || command == 'H') {
                float h, r, p;
                imu_read(&h, &r, &p);
                printf("Heading: %.1f | Roll: %.1f | Pitch: %.1f\n", h, r, p);
                success = true;
            }
            else if (command == 'q' || command == 'Q') {
                motors_stop();
                printf("Shutting down...\n");
                break;
            }
            
            // Send "done" response if command came from UART
            if (from_uart && (command == 'w' || command == 'W' || 
                             command == 's' || command == 'S' ||
                             command == 'a' || command == 'A' || 
                             command == 'd' || command == 'D')) {
                uart_send_string("done");
                printf(">>> Sent 'done' response <<<\n");
            }
        }
        
        sleep_ms(10);
    }
    return 0;
}