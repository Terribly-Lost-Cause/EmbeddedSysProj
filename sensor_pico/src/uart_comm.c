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
// PIO PROGRAMS (Copied from working comms_sense.c)
// =====================================================

// RX Program (Robust)
static const uint16_t pio_rx_instr[] = {
    // .wrap_target
    0x2020, //  0: wait   0 pin, 0                    ; Wait for Start bit (Low)
    0xea27, //  1: set    x, 7            [10]        ; Delay to middle of bit 0
    0x4001, //  2: in     pins, 1                     ; Sample data
    0x0642, //  3: jmp    x--, 2          [6]         ; Loop 8 times
    0x8020, //  4: push   block                       ; Push to FIFO
    // .wrap
};

static const struct pio_program pio_rx_program = {
    .instructions = pio_rx_instr, .length = 5, .origin = -1,
};

// TX Program (Robust - Explicit Set, No Side-Set)
static const uint16_t pio_tx_instr[] = {
    // .wrap_target
    0x80a0, //  0: pull   block                       ; Wait for data
    0xe027, //  1: set    x, 7                        ; Init bit counter
    0xe700, //  2: set    pins, 0         [7]         ; Start bit (Low)
    0x6001, //  3: out    pins, 1                     ; Data bit
    0x0643, //  4: jmp    x--, 3          [6]         ; Loop
    0xe701, //  5: set    pins, 1         [7]         ; Stop bit (High)
    // .wrap
};

static const struct pio_program pio_tx_program = {
    .instructions = pio_tx_instr, .length = 6, .origin = -1,
};

// =====================================================
// INIT
// =====================================================
void uart_comm_init(void) {
    // -----------------------------------------------------------------
    // 1. Setup RX (Read from GP6 via PIO)
    // -----------------------------------------------------------------
    offset_rx = pio_add_program(COMM_PIO, &pio_rx_program);
    sm_rx = pio_claim_unused_sm(COMM_PIO, true);

    // Pin Config
    pio_sm_set_consecutive_pindirs(COMM_PIO, sm_rx, COMM_RX_PIN, 1, false);
    pio_gpio_init(COMM_PIO, COMM_RX_PIN);
    gpio_pull_up(COMM_RX_PIN); // Critical for UART stability

    // SM Config
    pio_sm_config c_rx = pio_get_default_sm_config();
    sm_config_set_wrap(&c_rx, offset_rx + 0, offset_rx + 4);
    sm_config_set_in_pins(&c_rx, COMM_RX_PIN);
    sm_config_set_jmp_pin(&c_rx, COMM_RX_PIN);
    sm_config_set_in_shift(&c_rx, true, false, 32); // Right shift, no autopush
    sm_config_set_fifo_join(&c_rx, PIO_FIFO_JOIN_RX);
    
    // Clock Divider
    float div_rx = (float)clock_get_hz(clk_sys) / (8 * COMM_BAUD_RATE);
    sm_config_set_clkdiv(&c_rx, div_rx);
    
    pio_sm_init(COMM_PIO, sm_rx, offset_rx, &c_rx);
    pio_sm_set_enabled(COMM_PIO, sm_rx, true);

    // -----------------------------------------------------------------
    // 2. Setup TX (Send to GP7 via PIO)
    // -----------------------------------------------------------------
    offset_tx = pio_add_program(COMM_PIO, &pio_tx_program);
    sm_tx = pio_claim_unused_sm(COMM_PIO, true);

    // Pin Config
    pio_sm_set_pins_with_mask(COMM_PIO, sm_tx, 1u << COMM_TX_PIN, 1u << COMM_TX_PIN);
    pio_sm_set_pindirs_with_mask(COMM_PIO, sm_tx, 1u << COMM_TX_PIN, 1u << COMM_TX_PIN);
    pio_gpio_init(COMM_PIO, COMM_TX_PIN);
    gpio_put(COMM_TX_PIN, 1); // Set Idle High immediately

    // SM Config
    pio_sm_config c_tx = pio_get_default_sm_config();
    sm_config_set_wrap(&c_tx, offset_tx + 0, offset_tx + 5);
    
    // IMPORTANT: Map both OUT and SET to the TX pin
    sm_config_set_out_pins(&c_tx, COMM_TX_PIN, 1);
    sm_config_set_set_pins(&c_tx, COMM_TX_PIN, 1);
    
    sm_config_set_out_shift(&c_tx, true, false, 32); // Right shift
    sm_config_set_fifo_join(&c_tx, PIO_FIFO_JOIN_TX);
    
    // Clock Divider
    float div_tx = (float)clock_get_hz(clk_sys) / (8 * COMM_BAUD_RATE);
    sm_config_set_clkdiv(&c_tx, div_tx);

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
    printf("Sending Command: '%c' \n", c);
    // Send character
    pio_sm_put_blocking(COMM_PIO, sm_tx, (uint32_t)c);
    // Send newline (Crucial for protocol)
    pio_sm_put_blocking(COMM_PIO, sm_tx, (uint32_t)'\n');
}

void uart_send_string(const char *str) {
    if (!str) return;
    while (*str) {
        pio_sm_put_blocking(COMM_PIO, sm_tx, (uint32_t)*str);
        str++;
    }
    // Note: External code might expect a newline, adding it here based on previous file logic
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
                
                float px, py, pth;
                char status_msg_buffer[32]; // Buffer to capture the status string ("done", "none", etc.)

                // NEW: Scan for 4 items: PX, x, y, theta, status
                // We use %31s to safely capture the status string up to 31 chars
                if (sscanf(rx_line_buffer, "PX,%f,%f,%f,%31s", 
                           &px, &py, &pth, status_msg_buffer) == 4) {
                    
                    // 1. It is a POSE update! Update storage directly.
                    stored_pose_x = px;
                    stored_pose_y = py;
                    stored_pose_theta = pth;
                    stored_pose_valid = true;
                    
                    // 2. The status ("done" or "none") needs to be relayed to the main loop 
                    //    so the main loop can detect command completion.
                    if (strcmp(status_msg_buffer, "none") != 0) {
                        // Only enqueue if it's a significant status (like "done" or "error"),
                        // filtering out the constant "none" status updates.
                        enqueue_message(rx_line_buffer);
                    }

                } else {
                    // It is a normal, non-pose-related message (e.g. "error", "command received")
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