#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

// Include your existing libraries
#include "motors.h"
#include "imu.h"
#include "encoder.h"

// --- CONFIGURATION ---

// GPIO Pins for Encoders (Change these to match your wiring!)

#define ENC_PIN_LEFT 2
#define ENC_PIN_RIGHT 6

// Robot Physical Constants

// How many holes in your encoder disk? (Standard yellow motors usually have 20)

#define PULSES_PER_REV 20.0f

// Control Settings

#define LOOP_INTERVAL_MS 50 // Run control loop every 50ms (20Hz)

// CHANGED: Lowered speed to 35 RPM for "slowest possible" smooth movement

#define BASE_TARGET_RPM 35.0f

// --- TUNING KNOBS (PID) ---

// 1. Heading Correction (How strongly to correct steering)

// Higher = Snappier turning, but might wobble. Lower = Smoother, but slower fix.

#define KP_HEADING 1.5f

// 2. Speed Control (How aggressively to adjust Motor Power to match RPM)

// Converts RPM Error -> PWM change.

// e.g. 0.5 means "If we are 10 RPM slow, add 5% power"

#define KP_SPEED 0.05f

// --- GLOBAL VARIABLES ---

volatile uint32_t left_ticks = 0;

volatile uint32_t right_ticks = 0;

// Current Motor Power Estimates (0-100%)

float left_pwm_current = 0.0f;

float right_pwm_current = 0.0f;

// --- INTERRUPT SERVICE ROUTINE (Background Task) ---

// This runs automatically whenever a wheel turns, pausing the main code for a microsecond.

void gpio_callback(uint gpio, uint32_t events)
{

    if (gpio == ENC_PIN_LEFT)
    {

        left_ticks++;
    }
    else if (gpio == ENC_PIN_RIGHT)
    {

        right_ticks++;
    }
}

// --- HELPER MATH ---

float get_heading_error(float target, float current)
{

    float error = target - current;

    // Fix the 360 wrap-around

    if (error > 180.0f)
        error -= 360.0f;

    if (error < -180.0f)
        error += 360.0f;

    return error;
}

int main()
{

    stdio_init_all();

    sleep_ms(2000); // Wait for USB serial to connect

    printf("Initializing Robot Systems...\n");

    // 1. Initialize Hardware

    motors_init();

    imu_init();

    // 2. Initialize Encoder Pins with Interrupts

    gpio_init(ENC_PIN_LEFT);

    gpio_set_dir(ENC_PIN_LEFT, GPIO_IN);

    gpio_pull_up(ENC_PIN_LEFT);

    gpio_init(ENC_PIN_RIGHT);

    gpio_set_dir(ENC_PIN_RIGHT, GPIO_IN);

    gpio_pull_up(ENC_PIN_RIGHT);

    // Attach interrupts to catch every "Rising Edge" (0 -> 1 transition)

    gpio_set_irq_enabled_with_callback(ENC_PIN_LEFT, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    gpio_set_irq_enabled(ENC_PIN_RIGHT, GPIO_IRQ_EDGE_RISE, true); // Share same callback

    // --- NEW: Wait for User Input ---

    printf("\n>>> SYSTEM READY <<<\n");

    printf("Press 'w' to lock heading and start driving...\n");

    while (true)
    {

        int c = getchar_timeout_us(100000); // Check for input every 100ms

        if (c == 'w')
        {

            printf("Command received! Starting sequence...\n");

            break;
        }
    }

    // 3. Get Initial Heading (AFTER button press, so we lock the direction we are currently facing)

    // We average a few readings to set the "Straight" direction

    float initial_heading = 0;

    float r, p;

    printf("Locking current heading...\n");

    for (int i = 0; i < 10; i++)
    {

        float h;

        imu_read(&h, &r, &p);

        initial_heading += h;

        sleep_ms(20);
    }

    float target_heading = initial_heading / 10.0f;

    printf("Target Heading Locked: %.2f deg\n", target_heading);

    printf("Starting Motors at %.1f RPM...\n", BASE_TARGET_RPM);

    // Initial Kickstart Power (Guessing 40% starts the motors)

    left_pwm_current = 40.0f;

    right_pwm_current = 40.0f;

    // We set global speed to 0 because we will use 'motors_forward_bias'

    // to control the motors entirely with our calculated values.

    motors_set_speed(0);

    // Time tracking

    uint32_t last_loop_time = to_ms_since_boot(get_absolute_time());

    // --- MAIN LOOP ---

    while (true)
    {

        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        uint32_t dt = current_time - last_loop_time;

        if (dt >= LOOP_INTERVAL_MS)
        {

            // A. READ SENSORS

            // ---------------------------------------------------------

            // 1. Calculate RPM (Revolutions Per Minute)

            // Save ticks to local variable and reset counter atomically-ish

            uint32_t l_ticks_now = left_ticks;

            uint32_t r_ticks_now = right_ticks;

            left_ticks = 0;

            right_ticks = 0;

            // RPM = (Ticks / PulsesPerRev) * (60000ms / dt_ms)

            float left_rpm_actual = ((float)l_ticks_now / PULSES_PER_REV) * (60000.0f / dt);

            float right_rpm_actual = ((float)r_ticks_now / PULSES_PER_REV) * (60000.0f / dt);

            // 2. Get Heading

            float current_heading, roll, pitch;

            imu_read(&current_heading, &roll, &pitch);

            // B. OUTER LOOP: HEADING CORRECTION (The "Captain")

            // ---------------------------------------------------------

            float heading_error = get_heading_error(target_heading, current_heading);

            // Deadband: If error is tiny (< 2 degrees), ignore it to prevent jitter

            if (fabs(heading_error) < 2.0f)
                heading_error = 0;

            // Calculate RPM Adjustment

            // Correction > 0 means Turn Right (Left speeds up, Right slows down)

            float rpm_correction = heading_error * KP_HEADING;

            float left_target_rpm = BASE_TARGET_RPM + rpm_correction;

            float right_target_rpm = BASE_TARGET_RPM - rpm_correction;

            // C. INNER LOOP: SPEED CONTROL (The "Muscle")

            // ---------------------------------------------------------

            // Simple Incremental P-Controller:

            // "If I'm too slow, add power. If I'm too fast, subtract power."

            float left_rpm_error = left_target_rpm - left_rpm_actual;

            float right_rpm_error = right_target_rpm - right_rpm_actual;

            // Adjust PWM based on error

            left_pwm_current += left_rpm_error * KP_SPEED;

            right_pwm_current += right_rpm_error * KP_SPEED;

            // Safety Clamps (PWM must be 0-100)

            if (left_pwm_current > 100)
                left_pwm_current = 100;

            if (left_pwm_current < 0)
                left_pwm_current = 0;

            if (right_pwm_current > 100)
                right_pwm_current = 100;

            if (right_pwm_current < 0)
                right_pwm_current = 0;

            // D. APPLY TO MOTORS

            // ---------------------------------------------------------

            // Since motors_set_speed is 0, 'bias' acts as the absolute power setting.

            motors_forward_bias((int8_t)left_pwm_current, (int8_t)right_pwm_current);

            // E. DEBUGGING (Optional - view in Serial Monitor)

            // ---------------------------------------------------------

            // Format: Heading | Left Wheel | Right Wheel

            printf("Head: %.1f (Err: %.1f) | L_RPM: %.0f/%.0f (PWM:%.0f) | R_RPM: %.0f/%.0f (PWM:%.0f)\n",

                   current_heading, heading_error,

                   left_target_rpm, left_rpm_actual, left_pwm_current,

                   right_target_rpm, right_rpm_actual, right_pwm_current);

            last_loop_time = current_time;
        }
    }
}