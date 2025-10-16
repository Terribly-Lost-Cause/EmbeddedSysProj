#include "encoder.h"

void encoder_init(uint32_t encoder_pin) {
    gpio_init(encoder_pin);
    gpio_set_dir(encoder_pin, GPIO_IN);
    gpio_pull_up(encoder_pin); // Optional depending on sensor
}

bool encoder_check_pulse(uint32_t encoder_pin) {
    static int last_state_left = 1;
    static int last_state_right = 1;

    // Use separate static variables for each pin
    static uint32_t last_pin_checked = -1;
    int *last_state = (encoder_pin == 2) ? &last_state_left : &last_state_right;

    int current_state = gpio_get(encoder_pin);
    if (*last_state == 1 && current_state == 0) {
        *last_state = current_state;
        return true;
    }
    *last_state = current_state;
    return false;
}

float calculate_rpm(uint32_t pulses, uint32_t pulses_per_rev, float interval_sec) {
    float revolutions = (float)pulses / pulses_per_rev;
    float revolutions_per_sec = revolutions / interval_sec;
    return revolutions_per_sec * 60.0f;
}
