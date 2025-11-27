#ifndef UART_COMM_H
#define UART_COMM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Settings for the physical communication wires
#define UART_ID uart0
#define UART_TX_PIN 0       // Pin used to SEND data
#define UART_RX_PIN 1       // Pin used to RECEIVE data
#define UART_BAUD_RATE 9600 // Speed of communication

// The maximum number of letters we can store in one message
#define UART_MAX_MESSAGE_LEN 128

// --- List of Tools Available to Use ---

// Turn on the communication system (Must run this first!)
void uart_comm_init(void);


// --- Sending Data ---

// Send just one letter (adds a "new line" automatically)
void uart_send_char(char c);

// Send a full word or sentence (adds a "new line" automatically)
void uart_send_string(const char *str);

// Send raw data exactly as is, without adding a "new line"
void uart_send_raw(const char *data, size_t len);


// --- Receiving Data ---

// Check if a full message has arrived and copy it to your variable
// Returns 'true' if it found a message, 'false' if the mailbox is empty
bool uart_receive_message(char *buffer, size_t max_len);

// The background worker: You must call this inside your main loop
// It grabs data from the wires and pieces it together into messages
void uart_comm_process(void);

// Peek to see if a message is waiting (returns true/false)
// Useful if you want to know if data exists without reading it yet
bool uart_has_message(void);

#endif // UART_COMM_H