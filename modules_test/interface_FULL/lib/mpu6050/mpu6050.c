/**
 * @file mpu6050.c
 * @brief Interfaz para manejar el sensor MPU6050 utilizando I2C en el RP2040.
 */

#include <stdint.h>
#include <math.h>

#include "mpu6050.h"
#include "lib/I2C_driver/i2c_driver.h"


static uint8_t addr_mpu = 0x68; ///< Default I2C address for MPU6050
static i2c_inst_t *i2c_inst; ///< Either i2c0 or i2c1 instance. It is selected in mpu6050_init().
static mpu6050_accel_range_t current_acc_range = MPU6050_RANGE_2_G; ///< Current accelerometer range setting
static mpu6050_gyro_range_t current_gyro_range = MPU6050_RANGE_250_DEG; ///< Current gyroscope range setting

static float acc_factor = 1.0f/16384.0f; ///< Default accelerometer scale factor for +/- 2g (deafault range)
static float gyro_factor = 1.0f/131.0f; ///< Default gyroscope scale factor for +/- 250 deg/s (deafault range)



void mpu6050_init(i2c_inst_t *i2c, uint gpio_sda, uint gpio_scl, mpu6050_data_t *data) {
    i2c_inst = i2c; // Set the I2C instance
    i2c_setup(i2c, gpio_sda, gpio_scl); // Set up I2C pins

    // Power management 1 register
    i2c_write(i2c, addr_mpu, MPU6050_PWR_MGMT_1, 0x00); // Reset the device
    /* PWR_MGMT_1 
    ====================================================================
    BIT  Symbol    Description                                   Default
    ---  ------    --------------------------------------------- -------
    7   DEVICE_RESET  Device reset. When set to 1, all registers are reset to their default values.
    6   SLEEP_MODE   When set to 1, this bit puts the MPU-60X0 into sleep mode. 
    5   CYCLE        When set to 1, this bit enables cycle mode. In cycle mode, the device wakes up every other second.
    4   TEMP_DIS     When set to 1, this bit disables the temperature sensor
    3   CLKSEL       Clock source selection. This field selects the clock source for the MPU-60X0.
                     0: Internal 8MHz oscillator
                     1: PLL with X Gyro reference
                     2: PLL with Y Gyro reference
                     3: PLL with Z Gyro reference
                     4: External 32.768kHz reference
                     5: External 19.2MHz reference
    */
    

    sleep_ms(100); // Wait for device to stabilize (recommended by copilot)

    // get acc and gyro ranges
    getAccelerometerRange(&current_acc_range);
    getGyroRange(&current_gyro_range);

    switch(current_acc_range){
        case MPU6050_RANGE_2_G:
            acc_factor = 1.0f/16384.0f; // Scale factor for ±2g
            break;
        case MPU6050_RANGE_4_G:
            acc_factor = 1.0f/8192.0f; // Scale factor for ±4g
            break;
        case MPU6050_RANGE_8_G:
            acc_factor = 1.0f/4096.0f; // Scale factor for ±8g
            break;
        case MPU6050_RANGE_16_G:
            acc_factor = 1.0f/2048.0f; // Scale factor for ±16g
            break;
    }

    switch(current_gyro_range) {
        case MPU6050_RANGE_250_DEG:
            gyro_factor = 1.0f/131.0f; // Scale factor for ±250 degrees/second
            break;
        case MPU6050_RANGE_500_DEG:
            gyro_factor = 1.0f/65.5f; // Scale factor for ±500 degrees/second
            break;
        case MPU6050_RANGE_1000_DEG:
            gyro_factor = 1.0f/32.8f; // Scale factor for ±1000 degrees/second
            break;
        case MPU6050_RANGE_2000_DEG:
            gyro_factor = 1.0f/16.4f; // Scale factor for ±2000 degrees/second
            break;
    }

    data->acc_offset_x = 0.0f; // Initialize accelerometer offsets
    data->acc_offset_y = 0.0f;
    data->acc_offset_z = 0.0f;
    data->gyro_offset_x = 0.0f; // Initialize gyroscope offsets
    data->gyro_offset_y = 0.0f;
    data->gyro_offset_z = 0.0f;

    data->KalmanAngleRoll = 0.0f; // Initialize Kalman filter angles
    data->KalmanAnglePitch = 0.0f;
    data->KalmanAngleYaw = 0.0f; // Initialize Kalman filter angles
    data->KalmanUncertaintyAngleRoll = 2.0f * 2.0f; // Initialize Kalman uncertainties
    data->KalmanUncertaintyAnglePitch = 2.0f * 2.0f;
    data->KalmanUncertaintyAngleYaw = 2.0f * 2.0f;
}

void setAccelerometerRange(mpu6050_accel_range_t afs_sel) {
    // Accelerometer full scale range selection
    i2c_write(i2c_inst, addr_mpu, MPU6050_ACCEL_CONFIG, afs_sel << 3); // Set accelerometer range
    /* ACCEL_CONFIG 
    ====================================================================
    BIT  Symbol    Description                                   Default
    ---  ------    --------------------------------------------- -------
    3-4  AFS_SEL   Accelerometer full scale range selection.
                   00: ±2g
                   01: ±4g
                   10: ±8g
                   11: ±16g
    */
}

void getAccelerometerRange(mpu6050_accel_range_t *afs_sel) {
    uint8_t reg_value;
    i2c_read_n(i2c_inst, addr_mpu, (uint8_t[]){MPU6050_ACCEL_CONFIG}, &reg_value, 1); // Read accelerometer config register
    *afs_sel = (reg_value >> 3) & 0x03; // Extract AFS_SEL bits
}

void setFilterBandwidth(mpu6050_bandwidth_t bandwidth) {
    // Set digital low pass filter bandwidth
    i2c_write(i2c_inst, addr_mpu, MPU6050_CONFIG, bandwidth); // Write to CONFIG register
    /* CONFIG 
    ====================================================================
    BIT  Symbol    Description                                   Default
    ---  ------    --------------------------------------------- -------
    0-2  DLPF_CFG  Digital low pass filter configuration.
                   000: 260Hz
                   001: 184Hz
                   010: 94Hz
                   011: 44Hz
                   100: 21Hz
                   101: 10Hz
                   110: 5Hz
                   111: Reserved
    */
}

void setGyroRange(mpu6050_gyro_range_t range) {
    // Set gyroscope full scale range
    i2c_write(i2c_inst, addr_mpu, MPU6050_GYRO_CONFIG, range << 3); // Write to GYRO_CONFIG register
    /* GYRO_CONFIG 
    ====================================================================
    BIT  Symbol    Description                                   Default
    ---  ------    --------------------------------------------- -------
    3-4  FS_SEL    Gyroscope full scale range selection.
                   00: ±250 degrees/second
                   01: ±500 degrees/second
                   10: ±1000 degrees/second
                   11: ±2000 degrees/second
    */
}

void getGyroRange(mpu6050_gyro_range_t *range) {
    uint8_t reg_value;
    i2c_read_n(i2c_inst, addr_mpu, (uint8_t[]){MPU6050_GYRO_CONFIG}, &reg_value, 1); // Read gyroscope config register
    *range = (reg_value >> 3) & 0x03; // Extract FS_SEL bits
}
void mpu6050_read(mpu6050_data_t *data) {
    uint8_t buffer[6]; // Buffer to hold raw data from MPU6050

    i2c_read_n(i2c_inst, addr_mpu, (uint8_t[]){MPU6050_ACCEL_XOUT_H}, buffer, 6); // Read accelerometer data
    
    // These bytes combination will change in the LSM303D (GY-89 IMU).
    for (int i = 0; i < 3; i++) {
        data->accel[i] = ((buffer[i * 2] << 8) | buffer[i * 2 + 1]); // Combine high and low bytes
    }

    i2c_read_n(i2c_inst, addr_mpu, (uint8_t[]){MPU6050_GYRO_XOUT_H}, buffer, 6); // Read gyroscope data
    for (int i = 0; i < 3; i++) {
        data->gyro[i] = ((buffer[i * 2] << 8) | buffer[i * 2 + 1]); // Combine high and low bytes
    }

    data->AccX = ((float)data->accel[0] * acc_factor) + data->acc_offset_x; // Scale to g's
    data->AccY = ((float)data->accel[1] * acc_factor) + data->acc_offset_y;
    data->AccZ = ((float)data->accel[2] * acc_factor) + data->acc_offset_z;

    data->RateRoll = ((float)data->gyro[0] * gyro_factor) + data->gyro_offset_x; // Scale to degrees per second
    data->RatePitch = ((float)data->gyro[1] * gyro_factor) + data->gyro_offset_y;
    data->RateYaw = ((float)data->gyro[2] * gyro_factor) + data->gyro_offset_z; 

    //raw angles
    data->AngleRoll = atan(data->AccY / sqrt(data->AccX * data->AccX + data->AccZ * data->AccZ)) * 180.0f / M_PI; // Roll angle in degrees
    data->AnglePitch = -atan(data->AccX / sqrt(data->AccY * data->AccY + data->AccZ * data->AccZ)) * 180.0f / M_PI; // Pitch angle in degrees
    data->AngleYaw = 0.0f; // Yaw angle is not calculated
}

void mpu6050_calibrate_gyro(mpu6050_data_t *data, int samples) {
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;

    for (int i = 0; i < samples; i++) {
        mpu6050_read(data); // Read current gyro data
        sum_x += data->RateRoll; 
        sum_y += data->RatePitch;
        sum_z += data->RateYaw;
        sleep_ms(1); // Delay to allow for stable readings
    }

    // Calculate average offsets
    data->gyro_offset_x = - sum_x / samples;
    data->gyro_offset_y = - sum_y / samples;
    data->gyro_offset_z = - sum_z / samples;
}

void mpu6050_calibrate_acc(mpu6050_data_t *data, int samples) {
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    /**
     * TODO: Implement accelerometer calibration.
     * This function should read the accelerometer data multiple times
     */
}
