#ifndef UART_COMM_H
#define UART_COMM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// --- CONFIGURATION ---
// Virtual UART Settings (PIO)
#define COMM_PIO        pio0
#define COMM_RX_PIN     6       // Virtual RX (Connect to Other Pico TX)
#define COMM_TX_PIN     7       // Virtual TX (Connect to Other Pico RX)
#define COMM_BAUD_RATE  9600 

// Queue settings
#define UART_MAX_MESSAGE_LEN 128

// --- API ---

// 1. SETUP
// Initialize the PIO (Virtual UART) on pins 6 & 7
void uart_comm_init(void);

// 2. SENDING (TX)
void uart_send_char(char c);
void uart_send_string(const char *str);
void uart_send_raw(const char *data, size_t len);

// 3. RECEIVING (RX)
// Call this frequently in your main loop (Core 0)
// It handles incoming data, parses Pose ("PX..."), and enqueues other messages
void uart_comm_process(void);

// Check if a normal text message (non-Pose) is waiting
bool uart_has_message(void);

bool uart_receive_message(char *buffer, size_t max_len);

// 4. POSE DATA
// Returns true if we have valid pose data.
// This reads the latest values parsed by uart_comm_process.
bool pose_get_latest(float *x, float *y, float *theta);

#endif // UART_COMM_H