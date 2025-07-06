#include "i2c_driver.h"

void i2c_setup(i2c_inst_t *i2c, uint gpio_sda, uint gpio_scl) {
    i2c_init(i2c, 400 * 1000); // Initialize I2C at 400kHz
    gpio_set_function(gpio_sda, GPIO_FUNC_I2C);
    gpio_set_function(gpio_scl, GPIO_FUNC_I2C);
    gpio_pull_up(gpio_sda);
    gpio_pull_up(gpio_scl);
}

void i2c_write(i2c_inst_t *i2c,  uint8_t addr, uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data}; // Prepare buffer with register address and data
    i2c_write_blocking(i2c, addr, buf, 2, false); // Write data to I2C
}
void i2c_read_n(i2c_inst_t *i2c, uint8_t addr, uint8_t *reg_addr, uint8_t *data, size_t length) {

    i2c_write_blocking(i2c, addr, reg_addr, 1, true); // Write register address
    i2c_read_blocking(i2c, addr, data, length, false); // Read data from I2C
}
