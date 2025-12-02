#include "pico/stdlib.h"
#include <stdio.h>
#include "uart_comm1.h"   // PIO UART driver

int main() {
    stdio_init_all();
    sleep_ms(1500);

    printf("\n=== LIDAR PICO - PIO UART SEND TEST ===\n");
    printf("Using Virtual UART pins: RX=%d, TX=%d\n", COMM_RX_PIN, COMM_TX_PIN);

    // Initialise PIO virtual UART on 6/7
    uart_comm_init();

    int counter = 0;

    while (1) {
        char msg[64];
        sprintf(msg, "LIDAR_PING_%d", counter++);
        
        uart_send_string(msg);
        printf("Sent: %s\n", msg);

        sleep_ms(1000);
    }
}
