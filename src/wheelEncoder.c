#include "pico/stdlib.h"
#include "encoder.h"
#include <stdio.h>

const uint32_t LEFT_ENCODER_PIN = 2;
const uint32_t RIGHT_ENCODER_PIN = 6;
const uint32_t PULSES_PER_REV = 20;  // adjust to your encoder’s specs

int main() {
    stdio_init_all();
    encoder_init(LEFT_ENCODER_PIN);
    encoder_init(RIGHT_ENCODER_PIN);

    uint32_t left_count = 0, right_count = 0;
    uint32_t last_left = 0, last_right = 0;
    uint64_t last_time = time_us_64(); 

    while (true) {
        // Check left encoder
        if (encoder_check_pulse(LEFT_ENCODER_PIN)) {
            left_count++;
        }

        // Check right encoder
        if (encoder_check_pulse(RIGHT_ENCODER_PIN)) {
            right_count++;
        }

        uint64_t current_time = time_us_64();
        if (current_time - last_time >= 1000000) { // every 1 second
            uint32_t left_pulses = left_count - last_left;
            uint32_t right_pulses = right_count - last_right;

            float left_rpm = calculate_rpm(left_pulses, PULSES_PER_REV, 1.0f);
            float right_rpm = calculate_rpm(right_pulses, PULSES_PER_REV, 1.0f);

            printf("Left: %.2f RPM | Right: %.2f RPM", left_rpm, right_rpm);

            // Drift detection
            float diff = left_rpm - right_rpm;
            if (diff < 0) diff = -diff;

            float avg = (left_rpm + right_rpm) / 2.0f;
            if (avg > 0 && (diff / avg) > 0.1f) { // more than 10% mismatch
                printf(" ⚠️  Warning: Drift detected!\n");
            } else {
                printf(" ✅  Speeds matched.\n");
            }

            last_left = left_count;
            last_right = right_count;
            last_time = current_time;
        }
    }
}
