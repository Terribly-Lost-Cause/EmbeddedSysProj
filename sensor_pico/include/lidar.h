#ifndef LIDAR_H
#define LIDAR_H

#include <stdint.h>
#include <stdbool.h>

// Config
// 1 = use pose data in calculations (requires uart_comm)
#define USE_POSE_UART 1

// Filters
#define DIST_MIN_MM 10
#define DIST_MAX_MM 400

// Motor control
void motor_start(float duty_percent);
void motor_set(float duty_percent);
void motor_stop(void);

// UART init (Hardware UART0 for LiDAR)
void uart_setup(void);       

// LIDAR commands
bool get_info(void);
bool start_scan(void);

// LIDAR node reading
bool read_one_node_resync(float *angle_deg, float *dist_mm, uint8_t *quality);

// Math Helpers
void polar_to_xy(float angle_deg, float dist_mm, float *x, float *y);
void local_to_world(float x_local, float y_local,
                    float robot_x, float robot_y, float theta_deg,
                    float *x_world, float *y_world);

#endif // LIDAR_H