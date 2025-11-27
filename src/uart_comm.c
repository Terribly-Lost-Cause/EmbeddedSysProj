#include "uart_comm.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <stdio.h>
#include <string.h>

// Internal buffer for receiving data
#define RX_BUFFER_SIZE 128
static char rx_buffer[RX_BUFFER_SIZE];
static int rx_index = 0;

// Message queue for received complete messages
#define MESSAGE_QUEUE_SIZE 8
static char message_queue[MESSAGE_QUEUE_SIZE][RX_BUFFER_SIZE];
static int message_queue_head = 0;
static int message_queue_tail = 0;
static int message_queue_count = 0;

void uart_comm_init(void) {
    // Initialize UART
    uart_init(UART_ID, UART_BAUD_RATE);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);
    
    // Set up GPIO pins
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    // Clear buffers
    rx_index = 0;
    memset(rx_buffer, 0, RX_BUFFER_SIZE);
    message_queue_head = 0;
    message_queue_tail = 0;
    message_queue_count = 0;
}

// Send a single character followed by newline
void uart_send_char(char c) {
    uart_putc_raw(UART_ID, c);
    uart_putc_raw(UART_ID, '\n');
}

// Send a null-terminated string followed by newline
void uart_send_string(const char *str) {
    if (str == NULL) return;
    
    // Send each character
    while (*str) {
        uart_putc_raw(UART_ID, *str);
        str++;
    }
    // Send newline terminator
    uart_putc_raw(UART_ID, '\n');
}

// Send raw data without newline
void uart_send_raw(const char *data, size_t len) {
    if (data == NULL || len == 0) return;
    
    for (size_t i = 0; i < len; i++) {
        uart_putc_raw(UART_ID, data[i]);
    }
}

// Add a message to the queue
static bool enqueue_message(const char *msg) {
    if (message_queue_count >= MESSAGE_QUEUE_SIZE) {
        return false;
    }
    
    strncpy(message_queue[message_queue_tail], msg, RX_BUFFER_SIZE - 1);
    message_queue[message_queue_tail][RX_BUFFER_SIZE - 1] = '\0';
    
    message_queue_tail = (message_queue_tail + 1) % MESSAGE_QUEUE_SIZE;
    message_queue_count++;
    
    return true;
}

// Get a message from the queue
static bool dequeue_message(char *buffer, size_t max_len) {
    if (message_queue_count == 0) {
        return false;
    }
    
    strncpy(buffer, message_queue[message_queue_head], max_len - 1);
    buffer[max_len - 1] = '\0';
    
    message_queue_head = (message_queue_head + 1) % MESSAGE_QUEUE_SIZE;
    message_queue_count--;
    
    return true;
}

// Check if there's a message waiting
bool uart_has_message(void) {
    return message_queue_count > 0;
}

// Receive a complete message
bool uart_receive_message(char *buffer, size_t max_len) {
    return dequeue_message(buffer, max_len);
}

// Process incoming UART data
void uart_comm_process(void) {
    while (uart_is_readable(UART_ID)) {
        char c = uart_getc(UART_ID);
        
        if (c == '\n') {
            // End of message
            rx_buffer[rx_index] = '\0';
            
            if (rx_index > 0) {
                // Add complete message to queue
                enqueue_message(rx_buffer);
            }
            
            rx_index = 0;
        } 
        else if (c >= 32 && c <= 126) {  // Printable ASCII only
            if (rx_index < RX_BUFFER_SIZE - 1) {
                rx_buffer[rx_index++] = c;
            } else {
                // Buffer overflow, reset
                rx_index = 0;
            }
        }
        // Ignore non-printable characters (except newline)
    }
}