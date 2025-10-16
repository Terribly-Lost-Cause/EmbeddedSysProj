#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>
#include <stdio.h>
#include "imu.h"

#define I2C_SDA 4
#define I2C_SCL 5
#define I2C_PORT i2c0
#define ACC_ADDR 0x19
#define MAG_ADDR 0x1E

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

    // Accelerometer setup
    i2c_write_reg(ACC_ADDR, 0x20, 0b01000111); // enable XYZ, 50 Hz
    i2c_write_reg(ACC_ADDR, 0x23, 0b00001000); // high-res ±2 g

    // Magnetometer setup
    i2c_write_reg(MAG_ADDR, 0x00, 0b00011000); // 15 Hz
    i2c_write_reg(MAG_ADDR, 0x01, 0b00100000); // gain
    i2c_write_reg(MAG_ADDR, 0x02, 0x00);       // continuous mode
}

void imu_read(float *heading, float *roll, float *pitch) {
    uint8_t raw[6];
    int16_t ax, ay, az, mx, my, mz;

    // --- Accel ---
    i2c_read_regs(ACC_ADDR, 0x28 | 0x80, raw, 6);
    ax = (int16_t)((raw[1] << 8) | raw[0]);
    ay = (int16_t)((raw[3] << 8) | raw[2]);
    az = (int16_t)((raw[5] << 8) | raw[4]);

    // --- Mag ---
    i2c_read_regs(MAG_ADDR, 0x03, raw, 6);
    mx = (int16_t)((raw[1] << 8) | raw[0]);
    mz = (int16_t)((raw[3] << 8) | raw[2]);
    my = (int16_t)((raw[5] << 8) | raw[4]);

    *heading = atan2f((float)my, (float)mx) * 180.0f / M_PI;
    if (*heading < 0) *heading += 360.0f;

    *roll  = atan2f((float)ay, (float)az) * 180.0f / M_PI;
    *pitch = atan2f(-(float)ax, sqrtf((float)ay * ay + (float)az * az)) * 180.0f / M_PI;
}
