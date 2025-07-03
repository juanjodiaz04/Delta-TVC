#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"

#include "lib/LSM303D/lsm303D.h"
#include "lib/mpu6050/mpu6050.h"
#include "lib/kalman_filter/kalman_filter.h"
/* Example code to talk to a MPU6050 MEMS accelerometer and gyroscope

   This is taking to simple approach of simply reading registers. It's perfectly
   possible to link up an interrupt line and set things up to read from the
   inbuilt FIFO to make it more useful.
*/
#define I2C_PORT i2c0
#define I2C_SDA 20
#define I2C_SCL 21
#define UPDATE_RATE_MS 10  // Update rate in milliseconds
#define UPDATE_RATE_S (UPDATE_RATE_MS / 1000.0f)  // Update rate in seconds
// By default these devices  are on bus address 0x68

/**
 * TODO: Si nos cambiamos a GY-89, es necesario pasar estas funciones de su giroscopio
 * a una librería ordenada.
 * Ignorar de momento.
 */
static int addr_L3GD20 = 0x6B; // L3GD20 gyroscope address
float RateRoll, RatePitch, RateYaw;
void who_am_i_l3gd20() {
    // Read the device ID to check we are talking to the right device
    uint8_t buffer[1];
    uint8_t val = 0x0F; // Who am I register for L3GD20
    i2c_write_blocking(i2c_default, addr_L3GD20, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, addr_L3GD20, buffer, 1, false); // False - finished with bus
    printf("L3GD20 Device ID = 0x%02X\n", buffer[0]);
}

void setup_l3gd20() {
    // Reset the L3GD20
    uint8_t buf[] = {0x20, 0x0F}; // CTRL_REG1 register, normal mode and all axes enabled
    i2c_write_blocking(i2c_default, addr_L3GD20, buf, 2, false);
    
    buf[0] = 0x23; // CTRL_REG4 register
    buf[1] = 0x00; // GYRO_RANGE_250DPS
    i2c_write_blocking(i2c_default, addr_L3GD20, buf, 2, false);

    who_am_i_l3gd20(); // Check we are talking to the right device
}

void l3gd20_read_raw(int16_t gyro[3]) {
    uint8_t buffer[6];

    // Read gyroscope data from register 0x28 for 6 bytes
    uint8_t val = 0x28 | 0x80; // Auto increment bit set
    i2c_write_blocking(i2c_default, addr_L3GD20, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, addr_L3GD20, buffer, 6, false);

    gyro[0] = (int16_t)(buffer[0] | (buffer[1] << 8));
    gyro[1] = (int16_t)(buffer[2] | (buffer[3] << 8));
    gyro[2] = (int16_t)(buffer[4] | (buffer[5] << 8));

    //printf("Gyro X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);

    // Scale data to degrees per second
    RateRoll = (float)gyro[0] * 0.00875; // Scale to degrees per second
    RatePitch = (float)gyro[1] * 0.00875;
    RateYaw = (float)gyro[2] * 0.00875;

    printf("Scaled Rate Roll = %f d/s, Y = %f deg/s, Z = %f deg/s\n", RateRoll, RatePitch, RateYaw);
}

// void kalman_1d(float *KalmanState, float *KalmanUncertainty, float KalmanInput, float KalmanMeasurement, float dt) {
//   // kalmanState = Angle
//   // KalmanInput = Input Variable => Rate
//   *KalmanState = *KalmanState + dt * KalmanInput;
//   *KalmanUncertainty = *KalmanUncertainty + dt * dt * 4 * 4; // supose std dev of rate = 4°/s 
//   float KalmanGain = *KalmanUncertainty * 1/(1 * *KalmanUncertainty + 3 * 3); //supose std dev angle = 3°
//   *KalmanState = *KalmanState + KalmanGain * (KalmanMeasurement - *KalmanState);
//   *KalmanUncertainty = (1 - KalmanGain) * (*KalmanUncertainty);

//   //float Kalman1DOutput[]={0,0}; // Output of the Kalman filter (angle, uncertainty)
// //   Kalman1DOutput[0] = KalmanState; 
// //   Kalman1DOutput[1] = KalmanUncertainty;
// }


/// Timer para capturar datos de IMU
static repeating_timer_t timer_inst_IMU;

/// Bandera para timer de captura de datos
volatile bool timer_flag_IMU = false;

/**
 * @brief Callback para el timer de captura de datos.
 * @param rt Puntero al timer que invoca la función.
 * @return true para repetir el timer.
 */
bool timer_callback_IMU(repeating_timer_t *rt) {
    timer_flag_IMU = true;
    return true;
}

int main() {
    stdio_init_all();
    sleep_ms(2000); // Wait for device to power up

    printf("Hello, MPU6050! Reading raw data from registers...\n");

    float RateCalibrationRoll = 0.0f, RateCalibrationPitch = 0.0f, RateCalibrationYaw = 0.0f;

    
    // lsm303D_data_t lsm303d_data; // Create a data structure for LSM303D
    // lsm303d_init(I2C_PORT, I2C_SDA, I2C_SCL, &lsm303d_data); // Initialize LSM303D

    // setup_l3gd20(); // Set up the L3GD20 gyroscope

    mpu6050_data_t mpu6050_data; // Create a data structure for MPU6050
    mpu6050_init(I2C_PORT, I2C_SDA, I2C_SCL, &mpu6050_data); // Initialize MPU6050
    mpu6050_calibrate_gyro(&mpu6050_data, 1000); // Calibrate gyroscope offsets

    printf("Calibration complete. Average rates:\n");
    printf("Roll: %f, Pitch: %f, Yaw: %f\n", 
            mpu6050_data.gyro_offset_x, mpu6050_data.gyro_offset_y, mpu6050_data.gyro_offset_z);

    int16_t acceleration[3], gyro[3], temp;
    printf("R:,P:,kR:,kP:\n");

    add_repeating_timer_ms(UPDATE_RATE_MS, timer_callback_IMU, NULL, &timer_inst_IMU);

    while (1) {

        // l3gd20_read_raw(gyro);

        // lsm303d_read(&lsm303d_data); // Read data from LSM303D
        // printf("LSM303D Accel X = %f g, Y = %f g, Z = %f g\n", 
        //        lsm303d_data.AccX, lsm303d_data.AccY, lsm303d_data.AccZ);
        
        if(timer_flag_IMU){
            timer_flag_IMU = false;
            mpu6050_read(&mpu6050_data); // Read data from MPU6050
            // printf("%f,%f,%f,", 
            //        mpu6050_data.AccX, mpu6050_data.AccY, mpu6050_data.AccZ);
            printf("%f,%f,", 
                mpu6050_data.AngleRoll, mpu6050_data.AnglePitch);

            kalman_1d(&(mpu6050_data.KalmanAngleRoll), &(mpu6050_data.KalmanUncertaintyAngleRoll), 
                mpu6050_data.RateRoll, mpu6050_data.AngleRoll,
                UPDATE_RATE_S);
            // KalmanAngleRoll = Kalman1DOutput[0]; // Update Kalman angle
            // KalmanUncertaintyAngleRoll = Kalman1DOutput[1]; // Update Kalman uncertainty
            kalman_1d(&(mpu6050_data.KalmanAnglePitch), &(mpu6050_data.KalmanUncertaintyAnglePitch),
                mpu6050_data.RatePitch, mpu6050_data.AnglePitch,
                UPDATE_RATE_S);
            // KalmanAnglePitch = Kalman1DOutput[0]; // Update Kalman angle
            // KalmanUncertaintyAnglePitch = Kalman1DOutput[1]; // Update Kalman uncertainty
            
            printf("%f,%f\n", mpu6050_data.KalmanAngleRoll, mpu6050_data.KalmanAnglePitch);
        }
    }
}
