#include "pico/stdlib.h"
#include "pico/multicore.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h> // Added for strcmp

// =================================================================
// == INCLUDE YOUR DRIVER FILES HERE ==
// =================================================================

#include "lidar.h"
#include "mqtt.h"
#include "servo_mmwave_pan.h"
#include "uart_comm.h"

// =================================================================
// == GLOBAL VARIABLES (shared between cores) ==
// =================================================================

// Detection flags - volatile because they're shared between cores
volatile bool frontDet = false;
volatile bool backDet = false;
volatile bool leftDet = false;
volatile bool rightDet = false;

// Robot pose - volatile because updated by Core 1, read by Core 0
volatile float robot_x_mm = 0.0f;
volatile float robot_y_mm = 0.0f;
volatile float robot_theta_deg = 0.0f;
volatile bool pose_valid = false;
bool connected = false;

// =================================================================
// == HELPER FUNCTIONS ==
// =================================================================

bool initLidar() {
    // Spin motor up
    motor_start(95.0f);
    motor_set(95.0f);
    sleep_ms(1500);

    // GET_INFO
    if (!get_info()) {
        printf("GET_INFO failed, bumping PWM and retrying...\n");
        motor_set(100.0f);
        sleep_ms(800);
        if (!get_info()) {
            printf("Still no GET_INFO -> stopping\n");
            motor_stop();
            return false;
        }
    }
    printf("GET_INFO OK\n");

    // Start SCAN
    if (!start_scan()) {
        printf("SCAN start failed\n");
        motor_stop();
        return false;
    }
    printf("Lidar Booted!\n");
    return true;
}

// =================================================================
// == CORE 1: LIDAR TASK ==
// =================================================================

void lidar_task(void) {
    // Timestamps to track when we last saw an object in each zone
    uint32_t last_front_time = 0;
    uint32_t last_left_time  = 0;
    uint32_t last_right_time = 0;
    uint32_t last_back_time  = 0;
    
    // Timeout in milliseconds (approx 2 full rotations of the lidar)
    const uint32_t DETECTION_TIMEOUT = 300; 

    while(1) { 

        // Get current time for expiration checks
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // If (Now - LastSeen) > Timeout, clear the flag
        if (now - last_front_time > DETECTION_TIMEOUT) frontDet = false;
        if (now - last_left_time  > DETECTION_TIMEOUT) leftDet  = false;
        if (now - last_right_time > DETECTION_TIMEOUT) rightDet = false;
        if (now - last_back_time  > DETECTION_TIMEOUT) backDet  = false;


        // 2) read lidar node
        float angle, dist;
        uint8_t q;

        // We check the result of the read
        bool read_success = read_one_node_resync(&angle, &dist, &q);

        if (!read_success) {
            // printf("No scan detected!");
            continue; 
        }

        // 3) filter
        bool valid_detection_point = (dist >= DIST_MIN_MM && dist <= DIST_MAX_MM);
        bool valid_mapping_point = (dist >= 500);

        if (valid_detection_point) {
            // 4) Check for detections AND Update Timestamps
            if ((angle > 345.0f || angle < 15.0f)) {
                frontDet = true;
                last_front_time = now; 
            } else if (angle > 75.0f && angle < 105.0f) {
                leftDet = true;
                last_left_time = now;
            } else if (angle > 165.0f && angle < 195.0f) {
                backDet = true;
                last_back_time = now;
            } else if (angle > 255.0f && angle < 285.0f) {
                rightDet = true;
                last_right_time = now;
            }
        }

        // 5) Convert and Publish Data
        float x_local, y_local;
        float x_world, y_world;

        if (valid_mapping_point) {
            #if USE_POSE_UART
                // polar -> local XY (lidar frame)
                polar_to_xy(angle, dist, &x_local, &y_local);
                
                // convert to world
                local_to_world(x_local, y_local,
                            robot_x_mm, robot_y_mm, robot_theta_deg,
                            &x_world, &y_world);
                
                // send to your PC endpoint over TCP
                // Assuming publish_data is in mqtt.h
                if (connected) { publish_data(x_world, y_world); }
            #endif
        }
    }
}

// =================================================================
// == CORE 0: MAIN FUNCTION (STATE MACHINE) ==
// =================================================================

int main(void) {
    stdio_init_all();
    sleep_ms(2000); // Wait for serial monitor to connect

    // --- STATE 0: INITIALIZATION ---
    printf("=== Entering State 0: Initialization ===\n");
    
    // Initialize servo and mmWave sensor
    if (!servo_mmwave_init()) {
        printf("CRITICAL: Servo/mmWave init failed. Halting.\n");
    }
    
    // Initialize Wi-Fi and MQTT connection
    // (Assuming mqtt_init() is called inside servo_mmwave_init or not shown here)
    connected = mqtt_init(); 
    
    
    // Initialize Virtual UART (Comm + Pose)
    // CHANGED: Combined init. Replacing uart_pose_setup()
    uart_comm_init();
    printf("Virtual UART (Comms + Pose) initialized\n");
    
    // Initialize UART1 (LIDAR Hardware)
    uart_setup();
    printf("LiDAR UART initialized\n");

    // Initialize LiDAR hardware
    bool lidarInit = initLidar();
    if (!lidarInit) {
        printf("CRITICAL: Could not initialize LiDAR. Halting.\n");
        // while(1) sleep_ms(1000);
    }
   
    printf("=== State 0 Complete. Launching Core 1 for LiDAR task ===\n");
    
    // Launch LiDAR task on Core 1
    multicore_launch_core1(lidar_task);
    sleep_ms(500); // Give Core 1 time to start
    
    printf("=== Core 1 launched. Starting main FSM on Core 0 ===\n");

    // --- MAIN LOOP: STATE MACHINE (States 1-5) ---
    int current_state = 1;
    bool is_moving = false;
    bool mmwave_hit = false;
    bool waiting = false;
    bool command_done = false;

    while (1) {
        // Display current state and flags
        printf("State: %d Det flags: MMWave=%d, Front=%d, Left=%d, Right=%d, Back=%d\n", current_state, mmwave_hit, frontDet, leftDet, rightDet, backDet);

        // Process incoming UART messages (Reads Pose AND Comms)
        uart_comm_process();

        // Check for 'done' response from robo_pico
        char msg_buffer[UART_MAX_MESSAGE_LEN];
        while (uart_receive_message(msg_buffer, UART_MAX_MESSAGE_LEN)) {

            printf("--- RAW RX Buffer: [%s] ---\n", msg_buffer);
                    
            float px, py, pth;
            char status[32]; // Buffer to hold "done" or "none"

            // Parse: PX, x, y, theta, status
            // We expect 4 items. %31s reads the string at the end.
            if (sscanf(msg_buffer, "PX,%f,%f,%f,%31s", &px, &py, &pth, status) == 4) {
                
                // 1. Update Pose
                robot_x_mm = px;
                robot_y_mm = py;
                robot_theta_deg = pth;
                pose_valid = true;
                // printf("Pose: %.1f, %.1f | Status: %s\n", px, py, status);

                // 2. Check Status
                // We use strstr to be safe, but strcmp(status, "done") == 0 would also work
                if (strstr(status, "done") != NULL) {
                    waiting = false;
                    command_done = true;
                    printf(">>> Received 'done' from robo_pico <<<\n");
                }
            }
        }

        // --- Run State Logic ---
        switch (current_state) {
            
            // ===== STATE 1: FORWARD MOVEMENT WITH SCANNING =====
            case 1: {
                // Check mmWave sensor
                mmwave_hit = mmwave_check_target();
                
                // Pan the servo while moving
                servo_pan_task();
                
                // Check for obstacles
                if (frontDet || mmwave_hit) {
                    printf("OBSTACLE DETECTED! Stopping.\n");
                    
                    servo_stop();
                    is_moving = false;
                    mmwave_flush_buffer();
                    uart_send_char('Q');
                    
                    current_state = 2;
                    printf("State: %d Det flags: MMWave=%d, Front=%d, Left=%d, Right=%d, Back=%d\n", current_state, mmwave_hit, frontDet, leftDet, rightDet, backDet);
                    

                } else {
                    // printf("Move Forward!\n");
                    uart_send_char('W');
                }
                
                break;
            }
            
            // ===== STATE 2: OBSTACLE ANALYSIS & DECISION =====
            case 2: {
                sleep_ms(1000); // Brief pause before decision
                servo_stop();
                printf("State: %d Det flags: MMWave=%d, Front=%d, Left=%d, Right=%d, Back=%d\n", current_state, mmwave_hit, frontDet, leftDet, rightDet, backDet);

                // Decision logic based on LiDAR detections
                if (!leftDet && !rightDet) {
                    printf("  -> Both sides clear. Choosing LEFT.\n");
                    sleep_ms(50);// Brief pause before decision
                    current_state = 3;
                }
                else if (!leftDet && rightDet) {
                    printf("  -> Left clear, right blocked. Going LEFT.\n");
                    sleep_ms(50); // Brief pause before decision
                    current_state = 3;
                } else if (leftDet && !rightDet) {
                    printf("  -> Right clear, left blocked. Going RIGHT.\n");
                    sleep_ms(50); // Brief pause before decision
                    current_state = 4;
                } else {
                    printf("  -> Both sides blocked. REVERSING.\n");
                    sleep_ms(50); // Brief pause before decision
                    current_state = 5;
                }

                // Reset detection flags after decision
                frontDet = false;
                leftDet = false;
                rightDet = false;
                backDet = false;  
                break;
            }
            
            // ===== STATE 3: TURN LEFT =====
            case 3: {
                servo_stop();
                printf("  -> Executing 90-degree LEFT turn\n");
                
                // PHASE 1: SEND COMMAND
                if (!waiting && !command_done) {
                    printf("Turn Left  <<<<<<<<<<<<<\n");
                    uart_send_char('A'); // Send 'A' for Left
                    waiting = true;      // Start waiting for "done"
                }
                
                // PHASE 2: WAITING
                else if (waiting) {
                    // Do nothing, exit switch to let main loop check UART
                    break; 
                }
                
                // PHASE 3: CLEANUP
                else if (!waiting && command_done) {
                    printf("  -> LEFT turn complete\n");
                    
                    mmwave_flush_buffer();
                    frontDet = false;
                    
                    // Reset flag for next time
                    command_done = false; 
                    
                    current_state = 1; // Return to moving forward
                }


                break;
            }

            // ===== STATE 4: TURN RIGHT =====
            case 4: {
                servo_stop();
                // PHASE 1: SEND COMMAND
                if (!waiting && !command_done) {
                    printf("Turn Right >>>>>>>>>>>>>\n");
                    uart_send_char('D'); // Send 'D' for Right
                    waiting = true;
                }
                
                // PHASE 2: WAITING
                else if (waiting) {
                    // printf("Waiting for done");
                    break; 
                }
                
                // PHASE 3: CLEANUP
                else if (!waiting && command_done) {
                    printf("  -> RIGHT turn complete\n");
                    
                    mmwave_flush_buffer();
                    frontDet = false;
                    
                    command_done = false; 
                    
                    current_state = 1; // Return to moving forward
                }


                break;
            }

            // ===== STATE 5: REVERSE AND ROTATE =====
            case 5: {
                servo_stop();

                // uart_send_char('S');

                // if (!leftDet || !rightDet){
                //     uart_send_char('Q');
                // } 
                

                // mmwave_flush_buffer();
                
                // current_state = 2;
                

                // // PHASE 1: SEND COMMAND
                if (!waiting && !command_done) {
                    printf("Moving BACKWARD VVVVVVVVV\n");
                    uart_send_char('S'); // Send 'S' for Backward
                    waiting = true;
                }
                
                // PHASE 2: WAITING
                else if (waiting) {
                    // printf("Waiting for done");
                    break; 
                }
                
                // PHASE 3: CLEANUP
                else if (!waiting && command_done) {
                    printf("  -> Reverse complete\n");
                    
                    mmwave_flush_buffer();
                    frontDet = false;
                    
                    command_done = false; 
                    
                    // NOTE: Reverse usually goes back to State 2 (Decision) 
                    // to see if the path is clear now.
                    current_state = 2; 
                }

                break;
            }

            default: {
                printf("ERROR: Unknown state %d. Resetting to State 2.\n", current_state);
                current_state = 2;
                break;
            }
        }

        // Small delay to prevent overwhelming the loop
        sleep_us(100);
    }
}