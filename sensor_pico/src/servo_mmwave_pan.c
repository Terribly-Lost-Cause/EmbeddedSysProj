improved .c files
-------------------
#include "servo_mmwave_pan.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"

// ============================= Internal Types & Constants =============================

typedef enum { SMP_PAN_FORWARD, SMP_PAN_REVERSE } smp_pan_direction_t;

static const uint8_t SMP_FRAME_HEADER[] = {0xAA, 0xFF, 0x03, 0x00};
static const uint8_t SMP_FRAME_END[]    = {0x55, 0xCC};

#define SMP_FRAME_LENGTH        30
#define SMP_TARGET_DATA_LENGTH  8

typedef struct { 
    int16_t x_mm;
    int16_t y_mm;
    int16_t speed_cm_s; 
    bool detected; 
} smp_target_t;

// ============================= Internal State Variables =============================

static uint8_t              smp_frame_buffer[SMP_FRAME_LENGTH] = {0};
static uint8_t              smp_frame_pos = 0;
static smp_pan_direction_t  smp_current_dir   = SMP_PAN_FORWARD;
static float                smp_current_deg   = SMP_START_DEG;
static bool                 smp_in_edge_dwell = false;
static absolute_time_t      smp_edge_dwell_until;
static absolute_time_t      smp_last_update;

// ============================= Internal Helper Functions =============================

static inline uint16_t smp_angle_to_pwm_us(float angle_deg) {
    if (angle_deg < 0) angle_deg = 0;
    if (angle_deg > 180) angle_deg = 180;
    return (uint16_t)(500 + (angle_deg * 2000.0f) / 180.0f);
}

static void smp_set_servo_angle(float angle_deg) {
    uint16_t pulse_us = smp_angle_to_pwm_us(angle_deg);
    uint16_t level = (uint16_t)((pulse_us * 65535u) / 20000u);
    pwm_set_gpio_level(SERVO_PIN, level);
}

static void smp_servo_pwm_init_like_calibration(void){
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 64.0f);
    pwm_config_set_wrap(&cfg, 39062);
    pwm_init(slice, &cfg, true);
}

static inline int16_t smp_decode_value(uint16_t raw_val){
    if (raw_val & 0x8000) return (int16_t)(raw_val & 0x7FFF);
    else return (int16_t)(- (int16_t)raw_val);
}

static void smp_parse_target_data(const uint8_t* buf, smp_target_t* t){
    uint16_t rx, ry, rs;
    memcpy(&rx, buf, 2);
    memcpy(&ry, buf+2, 2);
    memcpy(&rs, buf+4, 2);
    t->x_mm = smp_decode_value(rx);
    t->y_mm = smp_decode_value(ry);
    t->speed_cm_s = smp_decode_value(rs);
    t->detected = (t->x_mm || t->y_mm || t->speed_cm_s);
}

static bool smp_is_zone_breached(const uint8_t* frame){
    smp_target_t t;
    for(int i=0; i<3; i++){
        smp_parse_target_data(&frame[4 + i * SMP_TARGET_DATA_LENGTH], &t);
        // Check Y distance (Forward) and ensure it's valid (>0)
        if (t.detected && t.y_mm > 0 && t.y_mm <= SMP_DANGER_ZONE_MM) return true;
    }
    return false;
}

// ============================= Public API Implementation =============================

bool servo_mmwave_init(void) {
    smp_servo_pwm_init_like_calibration();

    uart_init(SMP_UART_ID, SMP_BAUD_RATE);
    gpio_set_function(SMP_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(SMP_UART_RX_PIN, GPIO_FUNC_UART);

    // Enable FIFO (First In First Out) buffer
    uart_set_fifo_enabled(SMP_UART_ID, true);

    printf("SMP: Driver initialized on UART1. Start @70 deg.\n");

    smp_current_deg = SMP_START_DEG;
    smp_set_servo_angle(smp_current_deg);
    sleep_ms(1000);

    smp_last_update = get_absolute_time();
    smp_in_edge_dwell = false;

    return true;
}

bool mmwave_check_target(void) {
    // "Process what is in the buffer, but don't stay here forever"
    while (uart_is_readable(SMP_UART_ID)){
        uint8_t ch = uart_getc(SMP_UART_ID);

        if (smp_frame_pos < sizeof(SMP_FRAME_HEADER)){
            // Header matching state
            if (ch == SMP_FRAME_HEADER[smp_frame_pos]) {
                smp_frame_buffer[smp_frame_pos++] = ch;
            } else {
                smp_frame_pos = 0; // Reset if header doesn't match
            }
        } else {
            // Data collection state
            smp_frame_buffer[smp_frame_pos++] = ch;
            
            // Frame Complete Check
            if (smp_frame_pos == SMP_FRAME_LENGTH){
                bool tail_ok = (smp_frame_buffer[SMP_FRAME_LENGTH-2] == SMP_FRAME_END[0] &&
                                smp_frame_buffer[SMP_FRAME_LENGTH-1] == SMP_FRAME_END[1]);
                
                if (tail_ok){
                    bool zone = smp_is_zone_breached(smp_frame_buffer);
                    smp_frame_pos = 0; // Reset for next frame
                    if (zone) {
                        printf("SMP: Target Detected!\n");
                        return true; // EXIT IMMEDIATELY ON DETECTION
                    } 
                } else {
                    smp_frame_pos = 0; // Bad frame, reset
                }
            }
        }
    }
    return false; 
}

void servo_pan_task(void) {
    absolute_time_t now = get_absolute_time();

    if (smp_in_edge_dwell){
        if (!time_reached(smp_edge_dwell_until)){
            smp_set_servo_angle(smp_current_deg);
            return;
        } else {
            smp_in_edge_dwell = false;
            smp_last_update = now;
        }
    }

    float dt_s = (float)absolute_time_diff_us(smp_last_update, now) / 1e6f;
    smp_last_update = now;

    float step = SMP_SWEEP_SPEED_DPS * dt_s;
    if (step > SMP_MAX_STEP_PER_UPDATE) step = SMP_MAX_STEP_PER_UPDATE;

    if (smp_current_dir == SMP_PAN_FORWARD){ 
        smp_current_deg += step;
        if (smp_current_deg >= SMP_PAN_MAX_DEG){
            smp_current_deg = SMP_PAN_MAX_DEG;
            smp_edge_dwell_until = delayed_by_ms(now, SMP_EDGE_DWELL_MS);
            smp_in_edge_dwell = true;
            smp_current_dir = SMP_PAN_REVERSE;
        }
    } else { 
        smp_current_deg -= step;
        if (smp_current_deg <= SMP_PAN_MIN_DEG){
            smp_current_deg = SMP_PAN_MIN_DEG;
            smp_edge_dwell_until = delayed_by_ms(now, SMP_EDGE_DWELL_MS);
            smp_in_edge_dwell = true;
            smp_current_dir = SMP_PAN_FORWARD;
        }
    }

    smp_set_servo_angle(smp_current_deg);
}

void servo_stop(void) {
    smp_set_servo_angle(smp_current_deg);
    smp_in_edge_dwell = false;
    smp_last_update = get_absolute_time();
}

void mmwave_flush_buffer(void) {
    int count = 0;
    while (uart_is_readable(SMP_UART_ID) && count < 128){
        (void)uart_getc(SMP_UART_ID); 
        count++;
    }
    smp_frame_pos = 0; 
}