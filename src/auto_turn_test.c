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
#define SETTLE_TIME_MS 500
#define LEFT_ENCODER_PIN 2
#define TARGET_TURN_ANGLE 90.0f

// === TEST CONFIGURATION ===
#define RIGHT_TURN_COUNT 4
#define LEFT_TURN_COUNT 4
#define INITIAL_DELAY_MS 3000

static float get_angle_diff(float start, float current) {
    float diff = current - start;
    if (diff < -180.0f) diff += 360.0f;
    if (diff > 180.0f) diff -= 360.0f;
    return diff;
}

// Reset IMU readings to prevent drift accumulation
static void reset_imu_readings(void) {
    float h, r, p;
    for (int i = 0; i < 5; i++) {
        imu_read(&h, &r, &p);
        sleep_ms(30);
    }
}

static bool execute_right_turn(void) {
    float start_heading, roll, pitch;
    imu_read(&start_heading, &roll, &pitch);
    
    motors_set_speed(TURN_SPEED_PERCENT);
    motors_right();
    
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    float current_heading = start_heading;
    float turned_so_far = 0.0f;
    
    while (turned_so_far < TARGET_TURN_ANGLE) {
        imu_read(&current_heading, &roll, &pitch);
        
        float diff = get_angle_diff(start_heading, current_heading);
        turned_so_far = fabsf(diff);
        
        if (to_ms_since_boot(get_absolute_time()) - start_time > TURN_TIMEOUT_MS) {
            motors_stop();
            return false;
        }
    }
    
    motors_stop();
    
    // Reset IMU readings after turn to prevent drift
    sleep_ms(50);
    reset_imu_readings();
    
    return true;
}

static bool execute_left_turn(void) {
    float start_heading, roll, pitch;
    imu_read(&start_heading, &roll, &pitch);
    
    motors_set_speed(TURN_SPEED_PERCENT);
    motors_left();
    
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    float current_heading = start_heading;
    float turned_so_far = 0.0f;
    
    while (turned_so_far < TARGET_TURN_ANGLE) {
        imu_read(&current_heading, &roll, &pitch);
        
        float diff = get_angle_diff(start_heading, current_heading);
        turned_so_far = fabsf(diff);
        
        if (to_ms_since_boot(get_absolute_time()) - start_time > TURN_TIMEOUT_MS) {
            motors_stop();
            return false;
        }
    }
    
    motors_stop();
    
    // Reset IMU readings after turn to prevent drift
    sleep_ms(50);
    reset_imu_readings();
    
    return true;
}

int main() {
    stdio_init_all();
    
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    // Startup: 3 quick blinks
    for (int i = 0; i < 3; i++) {
        gpio_put(LED_PIN, 1); 
        sleep_ms(100);
        gpio_put(LED_PIN, 0); 
        sleep_ms(100);
    }
    
    motors_init();
    motors_set_speed(TURN_SPEED_PERCENT);
    imu_init();
    encoder_init(LEFT_ENCODER_PIN);
    
    // Initial IMU stabilization
    reset_imu_readings();
    
    // Initial delay - LED on during wait
    gpio_put(LED_PIN, 1);
    sleep_ms(INITIAL_DELAY_MS);
    gpio_put(LED_PIN, 0);
    
    // === PHASE 1: RIGHT TURNS ===
    for (int i = 0; i < RIGHT_TURN_COUNT; i++) {
        // Blink once before turn
        gpio_put(LED_PIN, 1); 
        sleep_ms(50);
        gpio_put(LED_PIN, 0);
        
        if (!execute_right_turn()) {
            // FAILURE: Rapid blink SOS pattern
            for (int j = 0; j < 10; j++) {
                gpio_put(LED_PIN, 1); sleep_ms(100);
                gpio_put(LED_PIN, 0); sleep_ms(100);
            }
            motors_stop();
            while(true) { sleep_ms(1000); }
        }
        
        sleep_ms(SETTLE_TIME_MS);
    }
    
    // Phase complete: 2 long blinks
    gpio_put(LED_PIN, 1); 
    sleep_ms(500);
    gpio_put(LED_PIN, 0);
    sleep_ms(300);
    gpio_put(LED_PIN, 1); 
    sleep_ms(500);
    gpio_put(LED_PIN, 0);
    
    sleep_ms(SETTLE_TIME_MS);
    
    // === PHASE 2: LEFT TURNS ===
    for (int i = 0; i < LEFT_TURN_COUNT; i++) {
        // Blink once before turn
        gpio_put(LED_PIN, 1); 
        sleep_ms(50);
        gpio_put(LED_PIN, 0);
        
        if (!execute_left_turn()) {
            // FAILURE: Rapid blink SOS pattern
            for (int j = 0; j < 10; j++) {
                gpio_put(LED_PIN, 1); sleep_ms(100);
                gpio_put(LED_PIN, 0); sleep_ms(100);
            }
            motors_stop();
            while(true) { sleep_ms(1000); }
        }
        
        sleep_ms(SETTLE_TIME_MS);
    }
    
    // === TEST COMPLETE ===
    motors_stop();
    
    // Success: 5 celebration blinks
    for (int i = 0; i < 5; i++) {
        gpio_put(LED_PIN, 1); 
        sleep_ms(200);
        gpio_put(LED_PIN, 0); 
        sleep_ms(200);
    }
    
    // Stay on to show completion
    gpio_put(LED_PIN, 1);
    
    while (true) {
        sleep_ms(1000);
    }
    
    return 0;
}
