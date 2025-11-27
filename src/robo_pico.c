#define PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS 5000

#include <stdio.h>
#include <string.h>
#include <math.h> 
#include "pico/stdlib.h"
#include "motors.h"
#include "encoder.h"
#include "imu.h"

// === SETTINGS ===
#define TURN_SPEED_PERCENT 35
#define TURN_TIMEOUT_MS 5000
#define SETTLE_TIME_MS 200
#define LEFT_ENCODER_PIN 2

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
    
    // Reset IMU readings to prevent drift accumulation
    float h, r, p;
    for (int i = 0; i < 5; i++) {
        imu_read(&h, &r, &p);
        sleep_ms(30);
    }
    
    sleep_ms(SETTLE_TIME_MS);
}

static bool execute_left_turn() {
    printf("\n╔════════════════════════════════════════╗\n");
    printf("║  EXECUTING LEFT TURN (IMU CONTROL)     ║\n");
    printf("╚════════════════════════════════════════╝\n");
    
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
    printf("\n╔════════════════════════════════════════╗\n");
    printf("║  EXECUTING RIGHT TURN (IMU CONTROL)    ║\n");
    printf("╚════════════════════════════════════════╝\n");
    
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
    printf("Turn Speed: %d%%\n", TURN_SPEED_PERCENT);
    printf("Commands:\n");
    printf("  'a' - Turn Left 90 Deg (IMU Auto-Stop)\n");
    printf("  'd' - Turn Right 90 Deg (IMU Auto-Stop)\n");
    printf("  'h' - Show Heading\n");
    printf("  'q' - Quit\n");
    
    motors_init();
    motors_set_speed(TURN_SPEED_PERCENT);
    
    imu_init();
    printf("IMU Initialized. Waiting for command...\n");
    
    encoder_init(LEFT_ENCODER_PIN);
    
    while (true) {
        int ch = getchar_timeout_us(0);
        
        // FIXED: Swapped function calls to match physical behavior
        if (ch == 'a' || ch == 'A') {
            execute_right_turn();  // 'a' physically turns left (calls motors_right)
        }
        else if (ch == 'd' || ch == 'D') {
            execute_left_turn();   // 'd' physically turns right (calls motors_left)
        }
        else if (ch == 'h' || ch == 'H') {
            float h, r, p;
            imu_read(&h, &r, &p);
            printf("Heading: %.1f | Roll: %.1f | Pitch: %.1f\n", h, r, p);
        }
        else if (ch == 'q' || ch == 'Q') {
            motors_stop();
            printf("Shutting down...\n");
            break;
        }
        
        sleep_ms(10);
    }
    return 0;
}
