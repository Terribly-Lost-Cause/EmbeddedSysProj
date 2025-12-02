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

void uart_comm_init(void);

// --- Sending Data ---
void uart_send_char(char c);
void uart_send_string(const char *str);
void uart_send_raw(const char *data, size_t len);

// --- Receiving Data ---
bool uart_receive_message(char *buffer, size_t max_len);
void uart_comm_process(void);
bool uart_has_message(void);

// --------------------------------------
// ADDED: Required by robo_pico.c
// --------------------------------------
char uart_comm_read_char(void);

#endif // UART_COMM_H
