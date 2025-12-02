#include "encoder.h"
#include <math.h>

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
    float rps = revolutions / interval_sec;
    return rps * 60.0f;
}



// ============================
// PULSE → DISTANCE
// ============================
float distance_from_pulses(int pulses, int pulses_per_rev, float wheel_radius) {
    float rev = (float)pulses / pulses_per_rev;
    return rev * 2.0f * M_PI * wheel_radius;
}



// ============================
// STANDARD ODOMETRY UPDATE
// ============================
void update_odometry(
    Pose *pose,
    int pulses_left,
    int pulses_right,
    int pulses_per_rev,
    float wheel_radius,
    float wheel_base
) {
    float dL = distance_from_pulses(pulses_left, pulses_per_rev, wheel_radius);
    float dR = distance_from_pulses(pulses_right, pulses_per_rev, wheel_radius);

    float d_center = 0.5f * (dL + dR);
    float d_theta  = (dR - dL) / wheel_base;

    float theta_mid = pose->theta + d_theta * 0.5f;

    pose->x     += d_center * cosf(theta_mid);
    pose->y     += d_center * sinf(theta_mid);
    pose->theta += d_theta;
}
