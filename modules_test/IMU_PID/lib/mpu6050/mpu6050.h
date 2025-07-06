#ifndef __MPU6050_H__
#define __MPU6050_H__

#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h" 


typedef struct {
    int16_t accel[3]; ///< Accelerometer data in raw format
    int16_t gyro[3]; ///< Gyroscope data in raw format
    float AccX; ///< Accelerometer X-axis data in g's
    float AccY; ///< Accelerometer Y-axis data in g's
    float AccZ; ///< Accelerometer Z-axis data in g's
    float RateRoll; ///< Gyroscope Roll rate in degrees per second
    float RatePitch; ///< Gyroscope Pitch rate in degrees per second
    float RateYaw; ///< Gyroscope Yaw rate in degrees per second
    float acc_offset_x; ///< Accelerometer X-axis offset
    float acc_offset_y; ///< Accelerometer Y-axis offset
    float acc_offset_z; ///< Accelerometer Z-axis offset
    float gyro_offset_x; ///< Gyroscope X-axis offset
    float gyro_offset_y; ///< Gyroscope Y-axis offset
    float gyro_offset_z; ///< Gyroscope Z-axis offset
    float AngleRoll; ///< Calculated Roll angle in degrees
    float AnglePitch; ///< Calculated Pitch angle in degrees
    float AngleYaw; ///< Calculated Yaw angle in degrees
    float KalmanAngleRoll; ///< Kalman filtered Roll angle
    float KalmanAnglePitch; ///< Kalman filtered Pitch angle
    
    float KalmanAngleYaw; ///< Kalman filtered Yaw angle
    float KalmanUncertaintyAngleRoll; ///< Kalman uncertainty for Roll angle
    float KalmanUncertaintyAnglePitch; ///< Kalman uncertainty for Pitch angle
    float KalmanUncertaintyAngleYaw; ///< Kalman uncertainty for Yaw angle
} mpu6050_data_t;

enum {
    MPU6050_CONFIG          = 0x1A, ///< Configuration register
    MPU6050_GYRO_CONFIG     = 0x1B, ///< Gyroscope configuration register
    MPU6050_ACCEL_CONFIG    = 0x1C, ///< Accelerometer configuration register
    MPU6050_FIFO_EN         = 0x23, ///< FIFO enable register
    MPU6050_I2C_MST_CTRL    = 0x24, ///< I2C Master control
    MPU6050_INT_PIN_CFG     = 0x37, ///< Interrupt pin configuration
    MPU6050_INT_ENABLE      = 0x38, ///< Interrupt enable register
    MPU6050_INT_STATUS      = 0x3A, ///< Interrupt status register
    MPU6050_ACCEL_XOUT_H    = 0x3B, ///< Accelerometer X-axis data high byte
    MPU6050_TEMP_OUT_H      = 0x41, ///< Temperature data high byte
    MPU6050_GYRO_XOUT_H     = 0x43, ///< Gyroscope X-axis data
    MPU6050_SIGNAL_PATH_RESET = 0x68, ///< Signal path reset register
    MPU6050_USER_CTRL       = 0x6A, ///< User control register
    MPU6050_PWR_MGMT_1      = 0x6B, ///< Power management 1 register
    MPU6050_PWR_MGMT_2      = 0x6C, ///< Power management 2 register
    MPU6050_WHO_AM_I        = 0x75, ///< Who am I register
};

// The following enums were taken from Adafruit's MPU6050 library for Arduino.
/**
 * @brief Accelerometer range options
 *
 * Allowed values for `setAccelerometerRange`.
 */
typedef enum {
  MPU6050_RANGE_2_G     = 0b00,  ///< +/- 2g (default value)
  MPU6050_RANGE_4_G     = 0b01,  ///< +/- 4g
  MPU6050_RANGE_8_G     = 0b10,  ///< +/- 8g
  MPU6050_RANGE_16_G    = 0b11, ///< +/- 16g
} mpu6050_accel_range_t;

/**
 * @brief Gyroscope range options
 *
 * Allowed values for `setGyroRange`.
 */
typedef enum {
  MPU6050_RANGE_250_DEG,  ///< +/- 250 deg/s (default value)
  MPU6050_RANGE_500_DEG,  ///< +/- 500 deg/s
  MPU6050_RANGE_1000_DEG, ///< +/- 1000 deg/s
  MPU6050_RANGE_2000_DEG, ///< +/- 2000 deg/s
} mpu6050_gyro_range_t;


/**
 * @brief Digital low pass filter bandthwidth options
 *
 * Allowed values for `setFilterBandwidth`.
 */
typedef enum {
  MPU6050_BAND_260_HZ, ///< Docs imply this disables the filter
  MPU6050_BAND_184_HZ, ///< 184 Hz
  MPU6050_BAND_94_HZ,  ///< 94 Hz
  MPU6050_BAND_44_HZ,  ///< 44 Hz
  MPU6050_BAND_21_HZ,  ///< 21 Hz
  MPU6050_BAND_10_HZ,  ///< 10 Hz
  MPU6050_BAND_5_HZ,   ///< 5 Hz
} mpu6050_bandwidth_t;

/**
 * @brief Accelerometer high pass filter options
 *
 * Allowed values for `setHighPassFilter`.
 */
typedef enum {
  MPU6050_HIGHPASS_DISABLE,
  MPU6050_HIGHPASS_5_HZ,
  MPU6050_HIGHPASS_2_5_HZ,
  MPU6050_HIGHPASS_1_25_HZ,
  MPU6050_HIGHPASS_0_63_HZ,
  MPU6050_HIGHPASS_UNUSED,
  MPU6050_HIGHPASS_HOLD,
} mpu6050_highpass_t;
//=============================================
/** TODO: 
 * I don't like these global variables, but they are used in the functions.
 * Maybe we can use another struct to hold the state of the MPU6050.
 * Auxiliar variables :u.
 */
// uint8_t addr_mpu = 0x68; ///< Default I2C address for MPU6050
// i2c_inst_t *i2c_inst; ///< Either i2c0 or i2c1 instance. It is selected in mpu6050_init().
// mpu6050_accel_range_t current_acc_range = MPU6050_RANGE_2_G; ///< Current accelerometer range setting
// mpu6050_gyro_range_t current_gyro_range = MPU6050_RANGE_250_DEG; ///< Current gyroscope range setting

// float acc_factor = 16384.0f; ///< Default accelerometer scale factor for +/- 2g (deafault range)
// float gyro_factor = 131.0f; ///< Default gyroscope scale factor for +/- 250 deg/s (deafault range)
//=============================================

/**
 * @brief Initialize the MPU6050 sensor.
 * 
 * This function initializes the MPU6050 sensor by setting up the I2C instance
 * and configuring the necessary registers for operation.
 * 
 * @param i2c Pointer to the I2C instance (either i2c0 or i2c1).
 * @param gpio_sda GPIO pin number for SDA.
 * @param gpio_scl GPIO pin number for SCL.
 */
void mpu6050_init(i2c_inst_t *i2c, uint gpio_sda, uint gpio_scl, mpu6050_data_t *data); 

/**
 * @brief Set the accelerometer full scale range.
 * This function configures the accelerometer's full scale range.
 * @param afs_sel The desired accelerometer full scale range.
 *                Options are defined in `mpu6050_accel_range_t`.
 */
void setAccelerometerRange(mpu6050_accel_range_t afs_sel); 

/**
 * @brief Get the current accelerometer full scale range.
 * This function retrieves the current accelerometer full scale range setting.
 * @param afs_sel Pointer to store the current accelerometer full scale range.
 */
void getAccelerometerRange(mpu6050_accel_range_t *afs_sel); 

/**
 * @brief Set the digital low pass filter bandwidth.
 * This function configures the digital low pass filter bandwidth.
 * @param bandwidth The desired bandwidth setting.
 *                  Options are defined in `mpu6050_bandwidth_t`.
 */
void setFilterBandwidth(mpu6050_bandwidth_t bandwidth); ///< Set digital low pass filter bandwidth

/**
 * @brief Set the high pass filter for the accelerometer.
 * This function configures the high pass filter settings for the accelerometer.
 * @param highpass The desired high pass filter setting.
 *                 Options are defined in `mpu6050_highpass_t`.
 */
void setGyroRange(mpu6050_gyro_range_t range); ///< Set gyroscope

/**
 * @brief Get the current gyroscope range.
 * This function retrieves the current gyroscope range setting.
 * @param range Pointer to store the current gyroscope range.
 */
void getGyroRange(mpu6050_gyro_range_t *range); ///< Get gyroscope range

/**
 * @brief Read data from the MPU6050 sensor.
 * This function reads accelerometer and gyroscope data from the MPU6050 sensor
 * and stores it in the provided `mpu6050_data_t` structure.
 * @param[out] data Pointer to the `mpu6050_data_t` structure where the read data will be stored.
 * @note The function reads raw data from the sensor and converts it to 
 * physical units (g's for accelerometer and degrees per second for gyroscope).
 */
void mpu6050_read(mpu6050_data_t *data);

/**
 * @brief Calibrate the gyroscope offsets.
 * This function calibrates the gyroscope by averaging multiple samples to determine
 * the offsets for each axis. The offsets are stored in the provided `mpu6050_data_t` structure.
 * @param data Pointer to the `mpu6050_data_t` structure where the offsets will be stored.
 * @param samples The number of samples to average for calibration.
 * 
 */
void mpu6050_calibrate_gyro(mpu6050_data_t *data, int samples);

void mpu6050_calibrate_accel(mpu6050_data_t *data, int samples);


//Suggested functions from the MPU6050_6Axis_MotionApps20.cpp

bool dmpInitialize(void); ///< Initialize the DMP (Digital Motion Processor)

void setIntEnabled(bool enabled); ///< Enable or disable interrupts

void setRate(uint8_t rate); ///< Set the sample rate divider



void setDMPEnabled(bool enabled); ///< Enable or disable the DMP

bool getDMPEnabled(void); ///< Check if the DMP is enabled

uint16_t dmpGetFIFOPacketSize(void); ///< Get the size of the DMP FIFO packet

uint8_t getIntStatus(void); ///< Get the interrupt status

uint16_t getFIFOCount(void); ///< Get the number of bytes in the FIFO buffer

void resetFIFO(void); ///< Reset the FIFO buffer

void getFIFOBytes(uint8_t *data, uint16_t length); ///< Read bytes from the FIFO buffer
#endif