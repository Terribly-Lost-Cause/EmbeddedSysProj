#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>
#include "uart_comm.h"   // Robo Pico UART (PIO UART)

int main() {
    stdio_init_all();
    sleep_ms(2000);

    printf("=== ROBO TEST START ===\n");

    // Init Robo Pico virtual UART (pins 0=TX, 1=RX or your actual pins!)
    uart_comm_init();

    char buffer[128];

    while (1) {
        uart_comm_process();

        if (uart_receive_message(buffer, sizeof(buffer))) {
            printf("ROBO RECEIVED: %s\n", buffer);
        }

        sleep_ms(10);
    }
}
