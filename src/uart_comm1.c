#include "uart_comm.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include <stdio.h>
#include <string.h>

// Internal State for PIO
static uint sm_rx = 0;
static uint sm_tx = 1;
static uint offset_rx;
static uint offset_tx;

// Buffer for building the current line being received
static char rx_line_buffer[UART_MAX_MESSAGE_LEN];
static int rx_line_index = 0;

// Message Queue (for non-pose messages)
#define MESSAGE_QUEUE_SIZE 8
static char message_queue[MESSAGE_QUEUE_SIZE][UART_MAX_MESSAGE_LEN];
static int message_queue_head = 0;
static int message_queue_tail = 0;
static int message_queue_count = 0;

// Pose Storage (Shared variables)
static volatile float stored_pose_x = 0.0f;
static volatile float stored_pose_y = 0.0f;
static volatile float stored_pose_theta = 0.0f;
static volatile bool  stored_pose_valid = false;

// =====================================================
// PIO PROGRAMS (The "Virtual Hardware")
// =====================================================

// RX Program
static const uint16_t pio_rx_instr[] = {
    0x2020, // 0: wait   0 pin, 0
    0xea27, // 1: set    x, 7           [10]
    0x4001, // 2: in     pins, 1
    0x0602, // 3: jmp    x--, 2         [6]
    0x2001, // 4: wait   1 pin, 0
    0x8020, // 5: push   block
};
static const struct pio_program pio_rx_program = {
    .instructions = pio_rx_instr, .length = 6, .origin = -1,
};

// TX Program
static const uint16_t pio_tx_instr[] = {
    0x9fa0, // 0: pull   block side 1   [7]
    0xe027, // 1: set    x, 7  side 0
    0x6001, // 2: out    pins, 1
    0x0002, // 3: jmp    x--, 2
};
static const struct pio_program pio_tx_program = {
    .instructions = pio_tx_instr, .length = 4, .origin = -1,
};

// =====================================================
// INIT
// =====================================================
void uart_comm_init(void) {
    // 1. Setup RX (Read from GP6)
    offset_rx = pio_add_program(COMM_PIO, &pio_rx_program);
    pio_sm_config c_rx = pio_get_default_sm_config();
    sm_config_set_in_pins(&c_rx, COMM_RX_PIN);
    sm_config_set_jmp_pin(&c_rx, COMM_RX_PIN);
    sm_config_set_in_shift(&c_rx, true, false, 32);
    sm_config_set_fifo_join(&c_rx, PIO_FIFO_JOIN_RX);
    float div_rx = (float)clock_get_hz(clk_sys) / (8 * COMM_BAUD_RATE);
    sm_config_set_clkdiv(&c_rx, div_rx);
    pio_gpio_init(COMM_PIO, COMM_RX_PIN);
    pio_sm_init(COMM_PIO, sm_rx, offset_rx, &c_rx);
    pio_sm_set_enabled(COMM_PIO, sm_rx, true);

    // 2. Setup TX (Send to GP7)
    offset_tx = pio_add_program(COMM_PIO, &pio_tx_program);
    pio_sm_config c_tx = pio_get_default_sm_config();
    sm_config_set_out_pins(&c_tx, COMM_TX_PIN, 1);
    sm_config_set_sideset_pins(&c_tx, COMM_TX_PIN);

    sm_config_set_sideset(&c_tx, 1, true, false);

    sm_config_set_out_shift(&c_tx, true, false, 32);
    sm_config_set_fifo_join(&c_tx, PIO_FIFO_JOIN_TX);
    float div_tx = (float)clock_get_hz(clk_sys) / (8 * COMM_BAUD_RATE);
    sm_config_set_clkdiv(&c_tx, div_tx);
    pio_gpio_init(COMM_PIO, COMM_TX_PIN);
    pio_sm_set_pins_with_mask(COMM_PIO, sm_tx, 1u << COMM_TX_PIN, 1u << COMM_TX_PIN);
    pio_sm_set_pindirs_with_mask(COMM_PIO, sm_tx, 1u << COMM_TX_PIN, 1u << COMM_TX_PIN);
    pio_sm_init(COMM_PIO, sm_tx, offset_tx, &c_tx);
    pio_sm_set_enabled(COMM_PIO, sm_tx, true);

    // 3. Clear Buffers
    rx_line_index = 0;
    message_queue_count = 0;
    printf("Virtual UART (PIO) initialized on Pins %d(RX)/%d(TX)\n", COMM_RX_PIN, COMM_TX_PIN);
}

// =====================================================
// SENDING
// =====================================================
void uart_send_char(char c) {
    // Only send if there is room in the FIFO
    if (!pio_sm_is_tx_fifo_full(COMM_PIO, sm_tx)) {
        pio_sm_put(COMM_PIO, sm_tx, (uint32_t)c);
    }
    
    if (!pio_sm_is_tx_fifo_full(COMM_PIO, sm_tx)) {
        pio_sm_put(COMM_PIO, sm_tx, (uint32_t)'\n');
    }
}
void uart_send_string(const char *str) {
    if (!str) return;
    while (*str) {
        pio_sm_put_blocking(COMM_PIO, sm_tx, (uint32_t)*str);
        str++;
    }
    pio_sm_put_blocking(COMM_PIO, sm_tx, (uint32_t)'\n');
}

void uart_send_raw(const char *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        pio_sm_put_blocking(COMM_PIO, sm_tx, (uint32_t)data[i]);
    }
}

// =====================================================
// RECEIVING & PROCESSING
// =====================================================

// Helper: Add normal message to queue
static void enqueue_message(const char *msg) {
    if (message_queue_count >= MESSAGE_QUEUE_SIZE) return;
    
    strncpy(message_queue[message_queue_tail], msg, UART_MAX_MESSAGE_LEN - 1);
    message_queue[message_queue_tail][UART_MAX_MESSAGE_LEN - 1] = '\0';
    
    message_queue_tail = (message_queue_tail + 1) % MESSAGE_QUEUE_SIZE;
    message_queue_count++;
}

void uart_comm_process(void) {
    // Read from PIO FIFO while data is available
    while (!pio_sm_is_rx_fifo_empty(COMM_PIO, sm_rx)) {
        uint32_t raw = pio_sm_get(COMM_PIO, sm_rx);
        char c = (char)(raw >> 24);

        if (c == '\n') {
            // End of line reached
            rx_line_buffer[rx_line_index] = '\0';
            
            if (rx_line_index > 0) {
                // DECIDE: Is this a Pose update or a Message?
                float px, py, pth;
                if (sscanf(rx_line_buffer, "PX,%f,%f,%f", &px, &py, &pth) == 3) {
                    // It is a POSE update! Update storage directly.
                    // We DO NOT enqueue this, to avoid flooding the message queue.
                    stored_pose_x = px;
                    stored_pose_y = py;
                    stored_pose_theta = pth;
                    stored_pose_valid = true;
                } else {
                    // It is a normal message (e.g. "done", "error")
                    enqueue_message(rx_line_buffer);
                }
            }
            rx_line_index = 0;
        } 
        else if (c >= 32 && c <= 126) {
            // Append char
            if (rx_line_index < UART_MAX_MESSAGE_LEN - 1) {
                rx_line_buffer[rx_line_index++] = c;
            } else {
                rx_line_index = 0; // Overflow, reset
            }
        }
    }
}

bool uart_has_message(void) {
    return message_queue_count > 0;
}

bool uart_receive_message(char *buffer, size_t max_len) {
    if (message_queue_count == 0) return false;
    
    strncpy(buffer, message_queue[message_queue_head], max_len - 1);
    buffer[max_len - 1] = '\0';
    
    message_queue_head = (message_queue_head + 1) % MESSAGE_QUEUE_SIZE;
    message_queue_count--;
    return true;
}

bool pose_get_latest(float *x, float *y, float *theta) {
    if (!stored_pose_valid) return false;
    
    *x = stored_pose_x;
    *y = stored_pose_y;
    *theta = stored_pose_theta;
    return true;
}