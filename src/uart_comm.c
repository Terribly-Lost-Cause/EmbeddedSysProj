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
    uart_init(UART_ID, UART_BAUD_RATE);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);
    
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    rx_index = 0;
    memset(rx_buffer, 0, RX_BUFFER_SIZE);
    message_queue_head = 0;
    message_queue_tail = 0;
    message_queue_count = 0;
}

void uart_send_char(char c) {
    uart_putc_raw(UART_ID, c);
    uart_putc_raw(UART_ID, '\n');
}

void uart_send_string(const char *str) {
    if (str == NULL) return;
    while (*str) {
        uart_putc_raw(UART_ID, *str);
        str++;
    }
    uart_putc_raw(UART_ID, '\n');
}

void uart_send_raw(const char *data, size_t len) {
    if (data == NULL || len == 0) return;
    for (size_t i = 0; i < len; i++) {
        uart_putc_raw(UART_ID, data[i]);
    }
}

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

bool uart_has_message(void) {
    return message_queue_count > 0;
}

bool uart_receive_message(char *buffer, size_t max_len) {
    return dequeue_message(buffer, max_len);
}

void uart_comm_process(void) {
    while (uart_is_readable(UART_ID)) {
        char c = uart_getc(UART_ID);
        
        if (c == '\n') {
            rx_buffer[rx_index] = '\0';
            
            if (rx_index > 0) {
                enqueue_message(rx_buffer);
            }
            
            rx_index = 0;
        } 
        else if (c >= 32 && c <= 126) {
            if (rx_index < RX_BUFFER_SIZE - 1) {
                rx_buffer[rx_index++] = c;
            } else {
                rx_index = 0;
            }
        }
    }
}

// --------------------------------------
// ADDED: Required by robo_pico.c
// Returns first character of next message
// --------------------------------------
char uart_comm_read_char(void) {
    char buf[UART_MAX_MESSAGE_LEN];
    if (uart_receive_message(buf, UART_MAX_MESSAGE_LEN)) {
        return buf[0];
    }
    return 0;
}
