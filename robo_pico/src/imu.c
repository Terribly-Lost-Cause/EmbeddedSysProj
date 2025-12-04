#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>
#include "imu.h"

#define I2C_SDA 4
#define I2C_SCL 5
#define I2C_PORT i2c0

// LSM303DLHC I2C Addresses
#define ACCEL_ADDR 0x19
#define MAG_ADDR 0x1E

// ==========================================
//  YOUR HARDCODED CALIBRATION VALUES
// ==========================================
// Hard-iron offsets (From your log)
static float mag_offset_x = 325.5f;
static float mag_offset_y = -47.0f;
static float mag_offset_z = 55.0f;

// Soft-iron scale factors (From your log)
static float mag_scale_x = 0.940f;
static float mag_scale_y = 0.941f;
static float mag_scale_z = 1.144f;

// Accelerometer offsets (From your log)
static float accel_offset_x = -1232.0f;
static float accel_offset_y = -112.0f;
static float accel_offset_z = 7560.0f;
// ==========================================

static void i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t data[2] = {reg, val};
    i2c_write_blocking(I2C_PORT, addr, data, 2, false);
}

static void i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
    i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buf, n, false);
}

void imu_init(void) {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    sleep_ms(200);
    
    // Configure Accelerometer
    i2c_write_reg(ACCEL_ADDR, 0x20, 0x57); // 100Hz, Normal
    sleep_ms(10);
    i2c_write_reg(ACCEL_ADDR, 0x23, 0x08); // High Res
    sleep_ms(10);
    
    // Configure Magnetometer
    i2c_write_reg(MAG_ADDR, 0x00, 0x18); // 75Hz
    sleep_ms(10);
    i2c_write_reg(MAG_ADDR, 0x01, 0x20); // Gain
    sleep_ms(10);
    i2c_write_reg(MAG_ADDR, 0x02, 0x00); // Continuous
    sleep_ms(10);
}

// Function kept in case you need to recalibrate later, but not needed now
void imu_calibrate(void) {
    printf("Using Hardcoded values. Re-upload code to change calibration logic if needed.\n");
}

void imu_read_raw_mag(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z) {
    uint8_t mag_data[6];
    i2c_read_regs(MAG_ADDR, 0x03 | 0x80, mag_data, 6);
    *mag_x = (int16_t)((mag_data[0] << 8) | mag_data[1]);
    *mag_z = (int16_t)((mag_data[2] << 8) | mag_data[3]);
    *mag_y = (int16_t)((mag_data[4] << 8) | mag_data[5]);
}

void imu_read(float *heading, float *roll, float *pitch) {
    uint8_t mag_data[6];
    uint8_t accel_data[6];
    
    // Read magnetometer
    i2c_read_regs(MAG_ADDR, 0x03 | 0x80, mag_data, 6);
    int16_t mag_x_raw = (int16_t)((mag_data[0] << 8) | mag_data[1]);
    int16_t mag_z_raw = (int16_t)((mag_data[2] << 8) | mag_data[3]);
    int16_t mag_y_raw = (int16_t)((mag_data[4] << 8) | mag_data[5]);
    
    // Apply hard-iron and soft-iron calibration
    float mag_x = (mag_x_raw - mag_offset_x) * mag_scale_x;
    float mag_y = (mag_y_raw - mag_offset_y) * mag_scale_y;
    float mag_z = (mag_z_raw - mag_offset_z) * mag_scale_z;
    
    // Read accelerometer
    i2c_read_regs(ACCEL_ADDR, 0x28 | 0x80, accel_data, 6);
    int16_t accel_x_raw = (int16_t)((accel_data[1] << 8) | accel_data[0]);
    int16_t accel_y_raw = (int16_t)((accel_data[3] << 8) | accel_data[2]);
    int16_t accel_z_raw = (int16_t)((accel_data[5] << 8) | accel_data[4]);
    
    // Apply accelerometer offsets
    float accel_x = accel_x_raw - accel_offset_x;
    float accel_y = accel_y_raw - accel_offset_y;
    float accel_z = accel_z_raw - accel_offset_z;
    
    // Calculate roll and pitch
    *roll = atan2f(accel_y, accel_z) * 180.0f / M_PI;
    *pitch = atan2f(-accel_x, sqrtf(accel_y*accel_y + accel_z*accel_z)) * 180.0f / M_PI;
    
    // Tilt compensation
    float roll_rad = (*roll) * M_PI / 180.0f;
    float pitch_rad = (*pitch) * M_PI / 180.0f;
    
    float mag_x_comp = mag_x * cosf(pitch_rad) + mag_z * sinf(pitch_rad);
    float mag_y_comp = mag_x * sinf(roll_rad) * sinf(pitch_rad) + 
                       mag_y * cosf(roll_rad) - 
                       mag_z * sinf(roll_rad) * cosf(pitch_rad);
    
    // Calculate heading
    *heading = atan2f(mag_y_comp, mag_x_comp) * 180.0f / M_PI;
    if (*heading < 0) *heading += 360.0f;
}