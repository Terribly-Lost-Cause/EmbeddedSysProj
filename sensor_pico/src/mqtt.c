#include "mqtt.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h" // Added for netif functions

#define WIFI_SSID "WIFINAME"
#define WIFI_PASSWORD "WIFIPASSWORD"
#define TCP_PORT 5000

// Holds the connection to your computer once it connects
static struct tcp_pcb *connected_client_pcb = NULL;

// Callback: Handle errors (client disconnects abruptly)
static void client_err(void *arg, err_t err) {
    printf("TCP Client error/disconnected: %d\n", err);
    connected_client_pcb = NULL; // Mark as disconnected
}

// Callback: Handle incoming data (or just keep connection alive)
static err_t client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (p == NULL) {
        // Client closed connection cleanly
        printf("Client disconnected.\n");
        tcp_close(tpcb);
        connected_client_pcb = NULL;
        return ERR_OK;
    }
    // We don't really expect data from the PC, but we must acknowledge it
    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}

// Callback: When a computer connects to the Pico
static err_t server_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
    if (err != ERR_OK || newpcb == NULL) {
        return ERR_VAL;
    }
    printf("Client connected!\n");
    
    // Save this connection so publish_data can use it
    connected_client_pcb = newpcb;
    
    // Setup callbacks for this new connection
    tcp_arg(newpcb, NULL);
    tcp_recv(newpcb, client_recv);
    tcp_err(newpcb, client_err);
    
    return ERR_OK;
}

// Initialize Wi-Fi and Start Listening on Port 5000
bool mqtt_init(void) {
    // 1. Initialize Wi-Fi Chip
    if (cyw43_arch_init()) {
        printf("WiFi init failed\n");
        return false;
    }
    cyw43_arch_enable_sta_mode();

    // 2. Connect to Wi-Fi
    printf("Connecting to WiFi SSID: %s\n", WIFI_SSID);
    int rc = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000);
    if (rc) {
        printf("WiFi connect failed with code %d\n", rc);
        return false;
    }
    printf("WiFi connected!\n");
    
    // 3. Create TCP Server
    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb) {
        printf("Failed to create TCP PCB\n");
        return false;
    }

    // Bind to Port 5000
    err_t err = tcp_bind(pcb, IP_ANY_TYPE, TCP_PORT);
    if (err != ERR_OK) {
        printf("Bind failed: %d\n", err);
        return false;
    }

    // Listen for connections
    pcb = tcp_listen_with_backlog(pcb, 1);
    if (!pcb) {
        printf("Listen failed\n");
        return false;
    }

    // Register accept callback
    tcp_accept(pcb, server_accept);

    // --- FIX: Get IP address correctly ---
    struct netif *n = &cyw43_state.netif[CYW43_ITF_STA];
    printf("TCP Server listening on %s port %d\n", ip4addr_ntoa(netif_ip4_addr(n)), TCP_PORT);
    
    return true;
}

// Send {"x":...,"y":...} to the connected computer
void publish_data(float x, float y) {
    // If no computer is connected, we can't send anything
    if (connected_client_pcb == NULL) {
        return;
    }

    char buf[64];
    int len = snprintf(buf, sizeof(buf), "{\"x\":%.2f,\"y\":%.2f}\n", x, y);

    // CRITICAL: We must lock the Wi-Fi stack before writing (Threadsafe)
    cyw43_arch_lwip_begin();
    
    // Check if buffer has space
    if (tcp_sndbuf(connected_client_pcb) > len) {
        err_t err = tcp_write(connected_client_pcb, buf, len, TCP_WRITE_FLAG_COPY);
        if (err == ERR_OK) {
            tcp_output(connected_client_pcb); // Push data immediately
        } else {
            // Optional debug
            // printf("TCP Write failed: %d\n", err);
        }
    }
    
    cyw43_arch_lwip_end();
}