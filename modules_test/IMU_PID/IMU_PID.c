#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#include "lib/mpu6050/mpu6050.h"
#include "lib/kalman_filter/kalman_filter.h"

#include "lib/servo_lib/servo_lib.h"
#include "lib/pid_lib/pid_lib.h"

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define UPDATE_RATE_MS 10  // Update rate in milliseconds
#define UPDATE_RATE_S (UPDATE_RATE_MS / 1000.0f)  // Update rate in seconds
// By default these devices  are on bus address 0x68

#define ROLL_SERVO_PIN 18
#define PITCH_SERVO_PIN 19

#define ROLL_OFFSET 80
#define PITCH_OFFSET 110

#define ROLL_SETPOINT 0
#define PITCH_SETPOINT 0

// Estructuras de datos
pid_controller_t pid_controller_roll;
pid_controller_t pid_controller_pitch;
servo_t servo_roll;
servo_t servo_pitch;
mpu6050_data_t mpu6050_data;

// Variables para las salidas del PID
float pid_output_roll = 0.0f;
float pid_output_pitch = 0.0f;

/// Timers
static repeating_timer_t timer_inst_IMU;
static repeating_timer_t timer_inst_PID;

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

bool PID_callback(repeating_timer_t *rt) {
    pid_compute(&pid_controller_roll);
    pid_compute(&pid_controller_pitch);
    // servo_set_angle(&servo_roll, (uint8_t)(*pid_controller_roll.output + ROLL_OFFSET));
    // servo_set_angle(&servo_pitch, (uint8_t)(*pid_controller_pitch.output + PITCH_OFFSET));
    servo_set_angle(&servo_roll, (uint8_t)(pid_output_roll + ROLL_OFFSET));
    servo_set_angle(&servo_pitch, (uint8_t)(pid_output_pitch + PITCH_OFFSET));
    //printf(" Roll:%f, Pitch:%f\n", *(pid_controller_roll.output) + ROLL_OFFSET, *(pid_controller_pitch.output) + PITCH_OFFSET);
    printf(" Roll:%f, Pitch:%f\n", pid_output_roll + ROLL_OFFSET, pid_output_pitch + PITCH_OFFSET);

    return true;
}

int main() {
    stdio_init_all();
    
    mpu6050_init(I2C_PORT, I2C_SDA, I2C_SCL, &mpu6050_data); // Initialize MPU6050
    servo_init(&servo_roll, ROLL_SERVO_PIN, 90);
    servo_init(&servo_pitch, PITCH_SERVO_PIN, 90);

    // pid_create(&pid_controller_roll, &mpu6050_data.KalmanAngleRoll, &servo_roll.angle, ROLL_SETPOINT, 100.0f, 0.0f, 0.0f, -20.0f, 20.0f);
    // pid_create(&pid_controller_pitch, &mpu6050_data.KalmanAnglePitch, &servo_pitch.angle, PITCH_SETPOINT, 100.0f, 0.0f, 0.0f, -30.0f, 30.0f);
    pid_create(&pid_controller_roll, &mpu6050_data.KalmanAngleRoll, &pid_output_roll, ROLL_SETPOINT, 
        30.0f, 3.0f, 0.0f, -20.0f, 20.0f);
    pid_create(&pid_controller_pitch, &mpu6050_data.KalmanAnglePitch, &pid_output_pitch, PITCH_SETPOINT, 
        30.0f, 3.0f, 0.0f, -30.0f, 30.0f);

    sleep_ms(2000); // Wait for device to power up

    printf("Hello, MPU6050! Reading raw data from registers...\n");

    float RateCalibrationRoll = 0.0f, RateCalibrationPitch = 0.0f, RateCalibrationYaw = 0.0f;

    mpu6050_calibrate_gyro(&mpu6050_data, 1000); // Calibrate gyroscope offsets

    printf("Calibration complete. Average rates:\n");
    printf("Roll: %f, Pitch: %f, Yaw: %f\n", 
            mpu6050_data.gyro_offset_x, mpu6050_data.gyro_offset_y, mpu6050_data.gyro_offset_z);

    int16_t acceleration[3], gyro[3], temp;
    printf("R:,P:,kR:,kP:\n");

    add_repeating_timer_ms(UPDATE_RATE_MS, timer_callback_IMU, NULL, &timer_inst_IMU);
    add_repeating_timer_ms(&pid_controller_roll.sampletime, PID_callback, NULL, &timer_inst_PID);

    while (1) {

        if(timer_flag_IMU){
            timer_flag_IMU = false;
            mpu6050_read(&mpu6050_data); // Read data from MPU6050

            kalman_1d(&(mpu6050_data.KalmanAngleRoll), &(mpu6050_data.KalmanUncertaintyAngleRoll), 
                mpu6050_data.RateRoll, mpu6050_data.AngleRoll,
                UPDATE_RATE_S);
            kalman_1d(&(mpu6050_data.KalmanAnglePitch), &(mpu6050_data.KalmanUncertaintyAnglePitch),
                mpu6050_data.RatePitch, mpu6050_data.AnglePitch,
                UPDATE_RATE_S);

            //printf("%f,%f,\n", mpu6050_data.AngleRoll, mpu6050_data.AnglePitch);
            
        }
    }
}
