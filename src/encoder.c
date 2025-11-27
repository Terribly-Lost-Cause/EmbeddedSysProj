#include "encoder.h"

// Prepare the specific pin to listen for wheel movement
void encoder_init(uint32_t encoder_pin) {
    gpio_init(encoder_pin);
    gpio_set_dir(encoder_pin, GPIO_IN); // Set as Input (Listening mode)
    gpio_pull_up(encoder_pin);          // specific sensor requirement
}

// Check if the wheel just moved a "step"
// This function returns 'true' ONLY at the exact moment the sensor triggers
bool encoder_check_pulse(uint32_t encoder_pin) {
    // These variables stay in memory even after the function finishes.
    // They remember what the sensor saw "last time" we checked.
    static int last_state_left = 1;
    static int last_state_right = 1;

    // Variable to track which pin was checked previously
    static uint32_t last_pin_checked = -1;
    
    // Decide which memory variable to use based on the pin number
    // If it's Pin 2, use the Left memory. Otherwise, use Right memory.
    int *last_state = (encoder_pin == 2) ? &last_state_left : &last_state_right;

    int current_state = gpio_get(encoder_pin);
    
    // Detect a "Falling Edge" (The moment the signal drops from High to Low)
    // This happens when a slot in the wheel passes the sensor.
    if (*last_state == 1 && current_state == 0) {
        *last_state = current_state; // Update memory
        return true;                 // Pulse detected!
    }
    
    // Update memory for next time, but return false (no new pulse yet)
    *last_state = current_state;
    return false;
}

// Standard math: Convert "pulses per second" into "Revolutions Per Minute"
float calculate_rpm(uint32_t pulses, uint32_t pulses_per_rev, float interval_sec) {
    float revolutions = (float)pulses / pulses_per_rev;
    float revolutions_per_sec = revolutions / interval_sec;
    return revolutions_per_sec * 60.0f;
}