#include "encoder.h"

// ============================
// ENCODER INITIALIZATION
// ============================
void encoder_init(uint32_t encoder_pin) {
    gpio_init(encoder_pin);
    gpio_set_dir(encoder_pin, GPIO_IN);

    // Pull-down gives stable behaviour for hall/IR encoders
    gpio_pull_down(encoder_pin);
}



// ============================
// UNIVERSAL EDGE DETECTION
// Counts a pulse on ANY CHANGE (0 -> 1 OR 1 -> 0)
// ============================
bool encoder_check_pulse(uint32_t pin) {
    static int last_left  = -1;
    static int last_right = -1;

    int current = gpio_get(pin);

    int *last = (pin == LEFT_ENCODER_PIN) ? &last_left : &last_right;

    // First reading: initialize state
    if (*last == -1) {
        *last = current;
        return false;
    }

    // ANY change counts as a pulse
    if (current != *last) {
        *last = current;
        return true;
    }

    *last = current;
    return false;
}



// ============================
// RPM CALCULATION
// ============================
float calculate_rpm(uint32_t pulses, uint32_t pulses_per_rev, float interval_sec) {
    float revolutions = (float)pulses / pulses_per_rev;
    float revolutions_per_sec = revolutions / interval_sec;
    return revolutions_per_sec * 60.0f;
}