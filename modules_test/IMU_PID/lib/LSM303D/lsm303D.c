#include <stdint.h>
#include "lsm303d.h"
#include "lib/I2C_driver/i2c_driver.h"

// Hardware interface
static i2c_inst_t *i2c_inst = NULL;              ///< I2C instance pointer
static uint8_t addr_LSM303D = 0x1D;               ///< LSM303D I2C address

// Current sensor configuration
static lsm303d_accel_range_t current_acc_range = LSM303D_RANGE_2_G;    ///< Current accelerometer range
static lsm303d_bandwidth_t current_bandwidth = LSM303D_BAND_773_HZ;    ///< Current filter bandwidth

static float acc_factor = LSM303D_LIN_ACC_FS_2G;              ///< Scale factor for accelerometer data

void lsm303d_init(i2c_inst_t *i2c, uint gpio_sda, uint gpio_scl, lsm303D_data_t *data) {
    i2c_inst = i2c; // Set the I2C instance
    i2c_setup(i2c, gpio_sda, gpio_scl); // Set up I2C pins

    // Reset the LSM303D
    i2c_write(i2c, addr_LSM303D, CTRL0, 0x00); // Reset CTRL0 register

    /**
     * CTRL0 register configuration
     * ====================================================================
     * BIT  Symbol    Description                                   Default
     * ---  ------    --------------------------------------------- -------
     * 7    BOOT      (0: normal mode; 1: reboot memory content)
     * 6    FIFO_EN   FIFO enable (0: disabled; 1: enabled)
     * 5    FTH       FIFO programmable threshold (0: disabled; 1: enabled)
     * 2    HP_Click  High-pass filter for click function (0: disabled; 1: enabled)
     * 1    HPIS1     High-pass filter for interrupt generator (0: disabled; 1: enabled)
     * 0    HPIS2     High-pass filter for interrupt generator (0: disabled; 1: enabled)
     */

    // CTRL1 register, BDU = 0, set to 100Hz, all axes enabled
    i2c_write(i2c, addr_LSM303D, CTRL1, 0xA7); 
    /**
     * CTRL1 register configuration
     * ====================================================================
     * BIT  Symbol    Description                                   Default
     * ---  ------    --------------------------------------------- -------
     * 7-4  AODR      Accelerometer output data rate
     *                  0000: Power down
     *                  0001: 3.125Hz
     *                  0010: 6.25Hz
     *                  0011: 12.5Hz ... etc.
     *                  0101: 50Hz ... (See datasheet)   
     * 3    BDU         Block data update (0: continuous; 1: output registers not updated until read)
     * 2    AZEN
     * 1    AYEN
     * 0    AXEN
     */

    // CTRL2 register. Anti alias filter bandwidth (0), Accelerometer full scale range (0)
    i2c_write(i2c, addr_LSM303D, CTRL2, 0x00);
    /**
     * CTRL2 register configuration
     * ====================================================================
     * BIT  Symbol    Description                                   
     * ---  ------    --------------------------------------------- 
     * 7-6  AFILT_BW  Anti-alias filter bandwidth (00: 773Hz; 01: 194Hz; 10: 362Hz; 11: 50Hz)
     * 5-4  AFS       Accelerometer full scale range (000: ±2g; 001: ±4g; 010: ±6g; 011: ±8g; 100: ±16g)
     * 3-0  
     */
    // CTRL3 register, Interrupts disabled
    i2c_write(i2c, addr_LSM303D, CTRL3, 0x00);

    // CTRL4 register, Interrupts and FIFO disabled
    i2c_write(i2c, addr_LSM303D, CTRL4, 0x00);

    lsm303d_get_accelerometer_range(&current_acc_range); // Get current accelerometer range

    switch (current_acc_range) {
        case LSM303D_RANGE_2_G:
            // Set scale factor for +/- 2g
            acc_factor = LSM303D_LIN_ACC_FS_2G;
            break;
        case LSM303D_RANGE_4_G:
            // Set scale factor for +/- 4g
            acc_factor = LSM303D_LIN_ACC_FS_4G;
            break;
        case LSM303D_RANGE_6_G:
            // Set scale factor for +/- 6g
            acc_factor = LSM303D_LIN_ACC_FS_6G;
            break;
        case LSM303D_RANGE_8_G:
            // Set scale factor for +/- 8g
            acc_factor = LSM303D_LIN_ACC_FS_8G;
            break;
        case LSM303D_RANGE_16_G:
            // Set scale factor for +/- 16g
            acc_factor = LSM303D_LIN_ACC_FS_16G;
            break;
    }
    // Initialize accelerometer offsets
    data->acc_offset_x = 0.0f; 
    data->acc_offset_y = 0.0f; 
    data->acc_offset_z = 0.0f; 
    
}

void lsm303d_set_accelerometer_range(lsm303d_accel_range_t range) {
    // Set accelerometer full scale range
    uint8_t current_reg_value;
    i2c_read_n(i2c_inst, addr_LSM303D, (uint8_t[]){CTRL2}, &current_reg_value, 1); // Read CTRL2 register
    //CF = 0b11001111 : Mask to clear AFS bits (bits 4 and 5)
    uint8_t reg_value = (current_reg_value & 0xCF) | ((range & 0x03) << 4); // Clear AFS bits and set new range
    i2c_write(i2c_inst, addr_LSM303D, CTRL2, reg_value); // Write to CTRL2 register
}

void lsm303d_get_accelerometer_range(lsm303d_accel_range_t *range) {
    uint8_t reg_value;
    i2c_read_n(i2c_inst, addr_LSM303D, (uint8_t[]){CTRL2}, &reg_value, 1); // Read CTRL2 register
    *range = (lsm303d_accel_range_t)((reg_value >> 4) & 0x03); // Extract AFS bits
}

void lsm303d_set_filter_bandwidth(lsm303d_bandwidth_t bandwidth) {
    // Set anti-alias filter bandwidth
    uint8_t reg_value = (bandwidth & 0x03) << 6; // Shift to correct position
    i2c_write(i2c_inst, addr_LSM303D, CTRL2, reg_value); // Write to CTRL2 register
}

void lsm303d_read(lsm303D_data_t *data) {
    uint8_t buffer[6];

    // Read accelerometer data from register OUT_X_L_A
    uint8_t reg_addr = OUT_X_L_A | 0x80; // Auto increment bit set
    i2c_read_n(i2c_inst, addr_LSM303D, &reg_addr, buffer, 6);

    for (int i = 0; i < 3; i++) {
        data->accel[i] = ((buffer[(i * 2) + 1] << 8) | buffer[i * 2] ); // Combine low and high bytes
    }

    // Convert raw data to g's
    data->AccX = ((float)data->accel[0] * acc_factor) + data->acc_offset_x;
    data->AccY = ((float)data->accel[1] * acc_factor) + data->acc_offset_y;
    data->AccZ = ((float)data->accel[2] * acc_factor) + data->acc_offset_z;
}