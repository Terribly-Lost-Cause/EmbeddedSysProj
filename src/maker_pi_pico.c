// Sender.c - Automatic sequence command sender via UART
#define PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS 5000

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "uart_comm.h"

// Timing
#define WAIT_FOR_DONE_TIMEOUT_MS 5000  // Wait up to 5 seconds for 'done'

// Send a command and wait for 'done' response
bool send_command_and_wait(char command) {
    char msg_buffer[UART_MAX_MESSAGE_LEN];
    
    printf("\n>>> SENDING: %c <<<\n", command);
    
    // Send command
    char cmd_str[2] = {command, '\0'};
    uart_send_string(cmd_str);
    
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    
    // Wait for 'done' response
    while (true) {
        uart_comm_process();
        
        while (uart_receive_message(msg_buffer, UART_MAX_MESSAGE_LEN)) {
            if (strcmp(msg_buffer, "done") == 0) {
                uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - start_time;
                printf(">>> RECEIVED: done (took %lu ms) <<<\n", elapsed);
                return true;
            }
        }
        
        // Check timeout
        uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - start_time;
        if (elapsed > WAIT_FOR_DONE_TIMEOUT_MS) {
            printf("!!! TIMEOUT: No 'done' response after %lu ms !!!\n", elapsed);
            return false;
        }
        
        sleep_ms(10);
    }
}

// Execute a sequence
void execute_sequence(int seq_num, char commands[], int num_commands) {
    printf("\n\n");
    printf("========================================\n");
    printf("=== SEQUENCE %d: ", seq_num);
    
    for (int i = 0; i < num_commands; i++) {
        printf("%s", commands[i] == 'A' ? "LEFT" : "RIGHT");
        if (i < num_commands - 1) printf(" -> ");
    }
    printf(" ===\n");
    printf("========================================\n");
    
    for (int i = 0; i < num_commands; i++) {
        printf("\nTurn %d/%d:\n", i + 1, num_commands);
        
        if (!send_command_and_wait(commands[i])) {
            printf("ERROR: Failed to complete turn %d\n", i + 1);
            return;
        }
        
        // Small pause between turns
        if (i < num_commands - 1) {
            printf("Pausing 500ms before next turn...\n");
            sleep_ms(500);
        }
    }
    
    printf("\n>>> SEQUENCE %d COMPLETE <<<\n", seq_num);
    printf("========================================\n\n");
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("\n\n");
    printf("********************************************\n");
    printf("*** SENDER: Autonomous Command Sender ***\n");
    printf("********************************************\n\n");

    printf("Initializing UART...\n");
    uart_comm_init();
    printf("  UART OK\n\n");
    
    // Set up LED
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    printf("Starting automatic sequence in 3 seconds...\n");
    printf("Make sure Receiver is powered on and ready!\n\n");
    sleep_ms(3000);
    
    gpio_put(LED_PIN, 1);
    
    // ========================================
    // SEQUENCE 1: Right > Right > Right > Right
    // ========================================
    char seq1[] = {'D', 'D', 'D', 'D'};
    execute_sequence(1, seq1, 4);
    
    printf("Pausing 3 seconds before next sequence...\n");
    sleep_ms(3000);
    
    // ========================================
    // SEQUENCE 2: Left > Left > Left > Left
    // ========================================
    char seq2[] = {'A', 'A', 'A', 'A'};
    execute_sequence(2, seq2, 4);
    
    printf("Pausing 3 seconds before next sequence...\n");
    sleep_ms(3000);
    
    // ========================================
    // SEQUENCE 3: Right > Left > Right > Left
    // ========================================
    char seq3[] = {'D', 'A', 'D', 'A'};
    execute_sequence(3, seq3, 4);
    
    gpio_put(LED_PIN, 0);
    
    printf("\n\n");
    printf("********************************************\n");
    printf("*** ALL SEQUENCES SENT! ***\n");
    printf("********************************************\n\n");
    
    while (true) {
        sleep_ms(1000);
    }
    
    return 0;
}
