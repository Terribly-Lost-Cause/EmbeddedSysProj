#include "lidar.h"
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"

// =====================================================
// CONFIG
// =====================================================

// 1. LIDAR (HARDWARE UART0)
#define LIDAR_UART_ID   uart0
#define LIDAR_TX_PIN    0       // GP0 -> LIDAR RX
#define LIDAR_RX_PIN    1       // GP1 <- LIDAR TX
#define LIDAR_BAUD      115200

// 2. MOTOR PWM
#define USE_MOTOR_PIN   1
#define PWM_PIN         16      

// LIDAR Timings
#define T_HDR_US        500000
#define T_NODE_US       200000

// =====================================================
// INTERNAL HELPERS
// =====================================================
static inline void send_command(uint8_t cmd) {
    uint8_t packet[2] = {0xA5, cmd};
    uart_write_blocking(LIDAR_UART_ID, packet, 2);
}

static bool read_exact(uint8_t *buf, int len, uint32_t timeout_us) {
    for (int i = 0; i < len; i++) {
        if (!uart_is_readable_within_us(LIDAR_UART_ID, timeout_us)) {
            return false;
        }
        buf[i] = uart_getc(LIDAR_UART_ID);
    }
    return true;
}

static void dump_hex(const char *tag, const uint8_t *b, int n) {
    printf("%s", tag);
    for (int i = 0; i < n; i++) {
        printf("%02X ", b[i]);
    }
    printf("\n");
}

// =====================================================
// MOTOR CONTROL
// =====================================================
void motor_start(float duty_percent) {
#if USE_MOTOR_PIN
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PWM_PIN);
    uint chan  = pwm_gpio_to_channel(PWM_PIN);

    pwm_set_wrap(slice, 9999);
    pwm_set_clkdiv(slice, 6.25f);

    if (duty_percent < 0.0f)  duty_percent = 0.0f;
    if (duty_percent > 100.0f) duty_percent = 100.0f;

    uint16_t level = (uint16_t)(duty_percent * 9999.0f / 100.0f);
    pwm_set_chan_level(slice, chan, level);
    pwm_set_enabled(slice, true);

    printf("Motor PWM started at %.1f%%\n", duty_percent);
#else
    (void)duty_percent;
    printf("Motor control disabled\n");
#endif
}

void motor_set(float duty_percent) {
#if USE_MOTOR_PIN
    if (duty_percent < 0.0f)  duty_percent = 0.0f;
    if (duty_percent > 100.0f) duty_percent = 100.0f;

    uint slice = pwm_gpio_to_slice_num(PWM_PIN);
    uint chan  = pwm_gpio_to_channel(PWM_PIN);
    uint16_t level = (uint16_t)(duty_percent * 9999.0f / 100.0f);
    pwm_set_chan_level(slice, chan, level);
#else
    (void)duty_percent;
#endif
}

void motor_stop(void) {
#if USE_MOTOR_PIN
    uint slice = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_set_enabled(slice, false);
#endif
}

// =====================================================
// UART SETUP
// =====================================================
void uart_setup(void) {
    // UART0 for Lidar (Pins 0/1)
    uart_init(LIDAR_UART_ID, LIDAR_BAUD);
    uart_set_format(LIDAR_UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(LIDAR_UART_ID, false, false);
    uart_set_fifo_enabled(LIDAR_UART_ID, true);
    gpio_set_function(LIDAR_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(LIDAR_RX_PIN, GPIO_FUNC_UART);

    while (uart_is_readable(LIDAR_UART_ID)) {
        (void)uart_getc(LIDAR_UART_ID);
    }
}

// =====================================================
// LIDAR COMMANDS
// =====================================================
bool get_info(void) {
    printf("Sending GET_INFO (0x50)\n");
    send_command(0x50);

    uint8_t hdr[7];
    if (!read_exact(hdr, 7, T_HDR_US)) {
        printf("GET_INFO: no header\n");
        return false;
    }
    dump_hex("INFO hdr: ", hdr, 7);

    uint8_t payload[20];
    if (!read_exact(payload, 20, T_HDR_US)) {
        printf("GET_INFO: header ok, payload missing\n");
        return false;
    }
    dump_hex("INFO payload: ", payload, 20);
    return true;
}

bool start_scan(void) {
    printf("Sending SCAN (0x20)\n");
    send_command(0x20);

    uint8_t hdr[7];
    if (!read_exact(hdr, 7, T_HDR_US)) {
        printf("SCAN: no 7B descriptor\n");
        return false;
    }
    dump_hex("SCAN hdr: ", hdr, 7);
    return true;
}

// =====================================================
// READ ONE NODE
// =====================================================
bool read_one_node_resync(float *angle_deg, float *dist_mm, uint8_t *quality) {
    uint8_t b0;
    uint32_t spin = 0;

    // find start byte
    while (true) {
        if (!uart_is_readable_within_us(LIDAR_UART_ID, T_NODE_US)) {
            return false;
        }
        b0 = uart_getc(LIDAR_UART_ID);

        bool s  = (b0 & 0x02) != 0;
        bool ns = (b0 & 0x01) == 0;
        if (s && ns) {
            break;
        }
        if ((++spin % 2000) == 0) {
            // printf("[resync] skipping\n");
        }
    }

    uint8_t rest[4];
    if (!read_exact(rest, 4, T_NODE_US)) {
        return false;
    }

    uint8_t d[5];
    d[0] = b0;
    d[1] = rest[0];
    d[2] = rest[1];
    d[3] = rest[2];
    d[4] = rest[3];

    *quality = d[0] >> 2;

    uint16_t angle_q6 = ((uint16_t)d[2] << 7) | ((uint16_t)d[1] >> 1);
    float a = angle_q6 / 64.0f;
    if (a >= 360.0f) a = fmodf(a, 360.0f);
    if (a < 0.0f)    a += 360.0f;
    *angle_deg = a;

    uint16_t dist_q2 = ((uint16_t)d[4] << 8) | d[3];
    *dist_mm = dist_q2 / 4.0f;
    

    return true;
}

// =====================================================
// MATH HELPERS
// =====================================================
void polar_to_xy(float angle_deg, float dist_mm, float *x, float *y) {
    float angle_rad = angle_deg * (3.14159265f / 180.0f);
    *x = dist_mm * cosf(angle_rad);
    *y = dist_mm * sinf(angle_rad);
}

void local_to_world(float x_local, float y_local,
                    float robot_x, float robot_y, float theta_deg,
                    float *x_world, float *y_world) {
    float th_rad = theta_deg * (3.14159265f / 180.0f);
    float c = cosf(th_rad);
    float s = sinf(th_rad);
    *x_world = robot_x + (x_local * c - y_local * s);
    *y_world = robot_y + (x_local * s + y_local * c);
}