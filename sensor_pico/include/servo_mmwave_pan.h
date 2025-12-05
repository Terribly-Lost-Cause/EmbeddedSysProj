#ifndef SERVO_MMWAVE_PAN_H
#define SERVO_MMWAVE_PAN_H

#include "pico/stdlib.h"
#include "hardware/uart.h"

/**
 * @file servo_mmwave_pan.h
 * @brief Public interface for the Servo-MMWave Panning Driver.
 *
 * This driver controls a servo motor to continuously sweep a mmWave radar
 * sensor and checks for target breaches in a predefined danger zone.
 */

// ============================= Configuration Macros =============================
// These macros MUST be defined by the user in their main application/config file
// before including this header. They define the hardware pins and the servo/radar behavior.

// --- Hardware Pins and UART Configuration ---
#define SMP_UART_ID             uart1           ///< The UART peripheral ID used (e.g., uart0, uart1)
#define SMP_BAUD_RATE           115200          ///< Baud rate for the mmWave sensor communication
#define SMP_UART_TX_PIN         4               ///< GPIO pin connected to the mmWave TX (Pico RX)
#define SMP_UART_RX_PIN         5               ///< GPIO pin connected to the mmWave RX (Pico TX)
#define SERVO_PIN               16              ///< GPIO pin connected to the servo motor PWM signal

// --- Panning (Sweep) Configuration ---
#define SMP_START_DEG           70.0f           ///< Initial servo angle (degrees, 0-180)
#define SMP_PAN_MIN_DEG         40.0f           ///< Minimum sweep angle
#define SMP_PAN_MAX_DEG         140.0f          ///< Maximum sweep angle
#define SMP_SWEEP_SPEED_DPS     20.0f           ///< Sweep speed in Degrees Per Second (DPS)
#define SMP_MAX_STEP_PER_UPDATE 2.0f            ///< Maximum angle step per update cycle (to prevent large jumps)
#define SMP_EDGE_DWELL_MS       500             ///< Time in milliseconds to pause at min/max angles

// --- mmWave Target Detection Configuration ---
#define SMP_DANGER_ZONE_MM      1500            ///< Target detection range in millimeters (e.g., 1500mm = 1.5m)
                                                ///< If a target's Y-coordinate is <= this distance, it triggers a breach.


// ============================= Public API Functions =============================

/**
 * @brief Initializes the UART and PWM peripherals.
 *
 * Sets up the servo motor PWM for a 50Hz period and initializes the UART
 * for communication with the mmWave sensor. Also sets the initial servo angle.
 *
 * @return true on success.
 */
bool servo_mmwave_init(void);

/**
 * @brief Reads the UART buffer and attempts to parse a full mmWave frame.
 *
 * Checks the received data against the frame header and end bytes. If a complete,
 * valid frame is received, it checks if any target falls within the defined
 * SMP_DANGER_ZONE_MM.
 *
 * @return true if a target is detected within the danger zone, false otherwise.
 */
bool mmwave_check_target(void);

/**
 * @brief Updates the servo motor's angle for continuous panning motion.
 *
 * This function should be called frequently within the main loop or a timer.
 * It calculates the next angle based on the sweep speed (SMP_SWEEP_SPEED_DPS)
 * and the time elapsed since the last update. It also handles the dwell time
 * (SMP_EDGE_DWELL_MS) at the min/max sweep edges.
 */
void servo_pan_task(void);

/**
 * @brief Stops the servo motion at its current position.
 *
 * Effectively stops the panning sweep but maintains the current angle.
 */
void servo_stop(void);

/**
 * @brief Flushes any remaining data from the UART receive buffer.
 *
 * Useful for clearing stale data after initialization or an error state.
 */
void mmwave_flush_buffer(void);

#endif // SERVO_MMWAVE_PAN_H