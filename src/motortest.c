#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "motors.h"

#define KILL_PIN 21   // GP21 is used as a physical kill switch (button to GND)

/* --------------------------------------------------------------------------
 *  Function: banner()
 *  Purpose : Prints control instructions to the Serial Monitor at startup.
 * -------------------------------------------------------------------------- */
static void banner(void){
    puts("=== Pico Motor Control ===");
    puts("Drive: w=FWD  s=BACK  a=LEFT  d=RIGHT  x=STOP");
    puts("Speed: 4..9 = 40..90%   0 = 100% (level 10)");
    puts("KILL : Hold GP21 button (to GND) to force STOP");
}

/* --------------------------------------------------------------------------
 *  Section: Print-on-change helpers
 *  These prevent repeated spam in the Serial Monitor when the Xbox controller
 *  continuously sends the same command (e.g., holding 'w' for forward).
 * -------------------------------------------------------------------------- */
static char last_dir = 0;        // Stores the last printed direction ('w','a','s','d','x')
static uint8_t last_speed = 0;   // Stores the last printed speed percentage

// Prints motor direction (FWD, BACK, etc.) only when it changes
static void print_dir_if_changed(char dir_code){
    if (dir_code == last_dir) return;  // do nothing if it's the same as before
    switch (dir_code) {
        case 'w': puts("FWD");   break;   // Forward
        case 's': puts("BACK");  break;   // Reverse
        case 'a': puts("LEFT");  break;   // Pivot left
        case 'd': puts("RIGHT"); break;   // Pivot right
        case 'x': puts("STOP");  break;   // Stop all motion
        default: break;
    }
    last_dir = dir_code;  // remember last printed direction
}

// Prints speed only when the percentage changes
static void print_speed_if_changed(uint8_t cur){
    if (cur != last_speed) {
        printf("SPEED %u%%\n", cur);
        last_speed = cur;
    }
}

/* --------------------------------------------------------------------------
 *  Function: main()
 *  Purpose : Entry point of the program. Handles:
 *              - Kill switch logic
 *              - Reading serial input from Xbox/keyboard
 *              - Calling motor control functions
 *              - Printing speed/direction updates
 * -------------------------------------------------------------------------- */
int main(void){
    stdio_init_all();     // Initialize USB serial I/O
    motors_init();        // Initialize PWM motor outputs

    // -------------------- Setup Hardware Kill Switch --------------------
    // The GP21 pin is configured as a pull-up input.
    // When the button is pressed (connected to GND), it reads LOW.
    gpio_init(KILL_PIN);
    gpio_pull_up(KILL_PIN);
    gpio_set_dir(KILL_PIN, GPIO_IN);

    sleep_ms(2000);             // Short delay to allow serial connection
    banner(); fflush(stdout);   // Display control instructions once

    bool was_kill = false;      // Tracks whether the kill switch was pressed previously

    // -------------------------- Main Loop -------------------------------
    while (true){
        /* ---------------------------------------------------------------
         *   Section: Kill switch check
         *   If GP21 is pressed, all motors stop immediately.
         *   While held, all keyboard/controller input is ignored.
         * --------------------------------------------------------------- */
        bool kill_now = (gpio_get(KILL_PIN) == 0);
        if (kill_now){
            if (!was_kill){
                puts("[KILL] GP21 pressed -> STOP (holding disables control)");
                was_kill = true;
                motors_stop();
                print_dir_if_changed('x');   // Print "STOP" once
            } else {
                motors_stop();  // continuously ensure motors are stopped
            }
            fflush(stdout);
            tight_loop_contents();
            continue; // Skip input handling while kill is active
        } else if (was_kill){
            puts("[KILL] GP21 released -> control restored");
            was_kill = false;
            fflush(stdout);
        }

        /* ---------------------------------------------------------------
         *   Section: Read user input (from serial / Xbox controller)
         *   getchar_timeout_us(0) reads one character at a time
         *   without blocking (non-blocking input).
         * --------------------------------------------------------------- */
        int ch = getchar_timeout_us(0);
        if (ch == PICO_ERROR_TIMEOUT) { tight_loop_contents(); continue; }

        switch (ch) {

            /* ---------------- Movement Commands ----------------
             * Each direction key sets motor state and prints the
             * direction only if it changed from the last.
             */
            case 'w': motors_forward();  print_dir_if_changed('w'); break;
            case 's': motors_backward(); print_dir_if_changed('s'); break;
            case 'a': motors_left();     print_dir_if_changed('a'); break;
            case 'd': motors_right();    print_dir_if_changed('d'); break;
            case 'x': motors_stop();     print_dir_if_changed('x'); break;

            /* ---------------- Speed Control --------------------
             * Keys 4–9 set 40–90% speed, and 0 sets 100%.
             * These commands come from both keyboard or Xbox
             * controller (stick magnitude and buttons).
             */
            case '4'...'9': {
                uint8_t pct = (ch - '0') * 10;                  // Convert ASCII digit to percentage
                motors_set_speed(pct);                          // Update motor PWM speed
                print_speed_if_changed(motors_get_speed());     // Print once per change
            } break;

            case '0': { // Level 10 (100%)
                motors_set_speed(100);
                print_speed_if_changed(motors_get_speed());
            } break;

            // Ignore Enter or other non-control characters
            case '\r': case '\n': break;
            default: break;
        }

        /* ---------------------------------------------------------------
         *   Section: Update Speed Display
         *   This ensures speed is printed if changed indirectly
         *   (e.g., via Xbox controller analog stick).
         * --------------------------------------------------------------- */
        print_speed_if_changed(motors_get_speed());
        fflush(stdout);
    }
}
