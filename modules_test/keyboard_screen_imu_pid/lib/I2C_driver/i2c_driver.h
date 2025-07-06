#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"


/**
 * @brief Read multiple bytes from an I2C device.
 * 
 * @param[in] i2c Pointer to the I2C instance (either i2c0 or i2c1).
 * @param[in] addr 7-bit I2C address of the device.
 * @param[in] reg_addr Pointer to the register address to read from.
 * @param[out] data Pointer to the buffer where the read data will be stored.
 * @param[in] length Number of bytes to read.
 * @note This function writes the register address to the device and then 
 * reads the specified number of bytes from it.
 */

void i2c_read_n(i2c_inst_t *i2c, uint8_t addr, uint8_t *reg_addr, uint8_t *data, size_t length);
/**
 * @brief Write a single byte to a specific register of an I2C device.
 * 
 * @param[in] i2c Pointer to the I2C instance (either i2c0 or i2c1).
 * @param[in] addr 7-bit I2C address of the device.
 * @param[in] reg Register address to write to.
 * @param[in] data Data byte to write to the register.
 */

void i2c_write(i2c_inst_t *i2c,  uint8_t addr, uint8_t reg, uint8_t data);

/**
 * @brief Set up the I2C instance with specified SDA and SCL GPIO pins.
 * 
 * @param i2c Pointer to the I2C instance (either i2c0 or i2c1).
 * @param gpio_sda GPIO pin number for SDA.
 * @param gpio_scl GPIO pin number for SCL.
 */
void i2c_setup(i2c_inst_t *i2c, uint gpio_sda, uint gpio_scl);



#endif