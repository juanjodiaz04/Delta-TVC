/**
 * @file lsm303D.h
 * @brief Header file for LSM303D accelerometer and magnetometer driver
 * This file contains definitions, structures, and function prototypes for interacting with the LSM303D sensor.
 * This driver is designed to work with the Raspberry Pi Pico and uses the I2C interface for communication.
 * @note Ensure that the I2C pins are correctly configured and that the sensor is powered appropriately.
 * In this case SA0 pin is connected to VCC, so the I2C address is 0x1D.
 * The default read data mode is set as "bypass mode" to allow direct access to the accelerometer data.
 * It might be implemented in the future to support FIFO mode.
 * According to the datasheet, acceleration data might slightly change due temperature and over time.
 */

#ifndef LSM303D_H
#define LSM303D_H

#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"


enum {
    WHO_AM_I  = 0x0F, ///< Who am I register
    CTRL0     = 0x1F, ///< Control register 0
    CTRL1     = 0x20, ///< Control register 1
    CTRL2     = 0x21, ///< Control register 2
    CTRL3     = 0x22, ///< Control register 3
    CTRL4     = 0x23, ///< Control register 4
    CTRL5     = 0x24, ///< Control register 5
    CTRL6     = 0x25, ///< Control register 6
    CTRL7     = 0x26, ///< Control register 7
    STATUS_A  = 0x27, ///< Status register A
    OUT_X_L_A = 0x28, ///< Accelerometer X-axis low byte
    FIFO_CTRL = 0x2E, ///< FIFO control register
    FIFO_SRC  = 0x2F, ///< FIFO source register
};
/**
 * @brief Linear Acceleration sensitivity constants
 *
 * Conversion factors for accelerometer data.
 * These factors are used to convert raw accelerometer data to g units.
 * Using multiplication factors (1/sensitivity) for better performance.
 */
#define LSM303D_LIN_ACC_FS_2G   (1.0f/16384.0f)  ///< Multiplication factor for +/- 2g
#define LSM303D_LIN_ACC_FS_4G   (1.0f/8192.0f)   ///< Multiplication factor for +/- 4g
#define LSM303D_LIN_ACC_FS_6G   (1.0f/5461.0f)   ///< Multiplication factor for +/- 6g
#define LSM303D_LIN_ACC_FS_8G   (1.0f/4096.0f)   ///< Multiplication factor for +/- 8g
#define LSM303D_LIN_ACC_FS_16G  (1.0f/2048.0f)   ///< Multiplication factor for +/- 16g

/**
 * @brief LSM303D Accelerometer data structure
 *
 * This structure holds the raw accelerometer data and its converted values in g's.
 * The raw data is stored as 16-bit integers, while the converted values are stored as floats.
 */
typedef struct {
    int16_t accel[3]; ///< Accelerometer data in raw format
    float AccX; ///< Accelerometer X-axis data in g's
    float AccY; ///< Accelerometer Y-axis data in g's
    float AccZ; ///< Accelerometer Z-axis data in g's
    float acc_offset_x; ///< Accelerometer X-axis offset
    float acc_offset_y; ///< Accelerometer Y-axis offset
    float acc_offset_z; ///< Accelerometer Z-axis offset
} lsm303D_data_t;

/**
 * @brief Accelerometer range options
 *
 * Allowed values for `setAccelerometerRange`.
 */
typedef enum {
    LSM303D_RANGE_2_G     = 0b000,  ///< +/- 2g (default value)
    LSM303D_RANGE_4_G     = 0b001,  ///< +/- 4g
    LSM303D_RANGE_6_G     = 0b010,  ///< +/- 6g
    LSM303D_RANGE_8_G     = 0b011,  ///< +/- 8g
    LSM303D_RANGE_16_G    = 0b100,  ///< +/- 16g
} lsm303d_accel_range_t;


/**
 * @brief Anti-alias filter bandwidth options
 *
 * Allowed values for `setFilterBandwidth`.
 */

typedef enum {
    LSM303D_BAND_773_HZ,  ///< 773 Hz
    LSM303D_BAND_362_HZ,  ///< 194 Hz
    LSM303D_BAND_194_HZ,  ///< 362 Hz
    LSM303D_BAND_50_HZ,   ///< 50 Hz
} lsm303d_bandwidth_t;
//=============================================
/** TODO: 
 * I don't like these global variables, but they are used in the functions.
 * Maybe we can use another struct to hold the state of the MPU6050.
 * Auxiliar variables :u.
 */

// //offsets
// float acc_offset_x = 0.0f; ///< Accelerometer X-axis offset
// float acc_offset_y = 0.0f; ///< Accelerometer Y-axis offset
// float acc_offset_z = 0.0f; ///< Accelerometer Z-axis offset
// i2c_inst_t *i2c_inst; ///< Pointer to the I2C instance (either i2c0 or i2c1)
// uint8_t addr_LSM303D = 0x1D; ///< Default I2C address for LSM303D
// lsm303d_accel_range_t current_acc_range = LSM303D_RANGE_2_G; ///< Current accelerometer range setting
// lsm303d_bandwidth_t current_bandwidth = LSM303D_BAND_773_HZ; ///< Current filter bandwidth setting
//=============================================

/**
 * @brief Initialize the LSM303D sensor
 * This function sets up the I2C instance and configures the LSM303D sensor.
 * @param i2c Pointer to the I2C instance (either i2c0 or i2c1).
 * @param gpio_sda GPIO pin number for SDA.
 * @param gpio_scl GPIO pin number for SCL.
 * @param[out] data Pointer to the `lsm303D_data_t` structure where the sensor data will be stored.
 */
void lsm303d_init(i2c_inst_t *i2c, uint gpio_sda, uint gpio_scl, lsm303D_data_t *data);

/**
 * @brief Read data from the LSM303D sensor
 * This function reads accelerometer data from the LSM303D sensor and stores it in the provided structure.
 * @param[out] data Pointer to the `lsm303D_data_t` structure where the read data will be stored.
 */
void lsm303d_read(lsm303D_data_t *data);

/**
 * @brief Set the accelerometer full scale range
 * This function configures the accelerometer's full scale range.
 * @param range The desired accelerometer full scale range.
 *              Options are defined in `lsm303d_accel_range_t`.
 */
void lsm303d_set_accelerometer_range(lsm303d_accel_range_t range);
/**
 * @brief Get the current accelerometer full scale range
 * This function retrieves the current accelerometer full scale range setting.
 * @param[out] range Pointer to store the current accelerometer full scale range.
 */
void lsm303d_get_accelerometer_range(lsm303d_accel_range_t *range);
/**
 * @brief Set the anti-alias filter bandwidth
 * This function configures the anti-alias filter bandwidth.
 * @param bandwidth The desired bandwidth setting.
 *                  Options are defined in `lsm303d_bandwidth_t`.
 */
void lsm303d_set_filter_bandwidth(lsm303d_bandwidth_t bandwidth);


#endif