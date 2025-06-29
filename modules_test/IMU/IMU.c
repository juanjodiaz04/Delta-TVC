#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#include "lib/LSM303D/lsm303D.h"
#include "lib/mpu6050/mpu6050.h"
/* Example code to talk to a MPU6050 MEMS accelerometer and gyroscope

   This is taking to simple approach of simply reading registers. It's perfectly
   possible to link up an interrupt line and set things up to read from the
   inbuilt FIFO to make it more useful.

   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefor I2C) cannot be used at 5v.

   You will need to use a level shifter on the I2C lines if you want to run the
   board at 5v.

   Connections on Raspberry Pi Pico board, other boards may vary.

   GPIO PICO_DEFAULT_I2C_SDA_PIN (On Pico this is GP4 (pin 6)) -> SDA on MPU6050 board
   GPIO PICO_DEFAULT_I2C_SCL_PIN (On Pico this is GP5 (pin 7)) -> SCL on MPU6050 board
   3.3v (pin 36) -> VCC on MPU6050 board
   GND (pin 38)  -> GND on MPU6050 board
*/
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9
// By default these devices  are on bus address 0x68
static int addr_mpu = 0x68;

static int addr_LSM303D = 0x1D; // LSM303D accelerometer and magnetometer address
static int addr_L3GD20 = 0x6B; // L3GD20 gyroscope address
float AccX, AccY, AccZ;
float RateRoll, RatePitch, RateYaw;

static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, addr_mpu, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, addr_mpu, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, addr_mpu, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, addr_mpu, &val, 1, true);
    i2c_read_blocking(i2c_default, addr_mpu, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, addr_mpu, &val, 1, true);
    i2c_read_blocking(i2c_default, addr_mpu, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];

    //==========================
    // These are the raw numbers from the chip, so will need tweaking to be really useful.
    // See the datasheet for more information
    AccX = (float)accel[0] / 16384.0; // Scale to g's
    AccY = (float)accel[1] / 16384.0;
    AccZ = (float)accel[2] / 16384.0;
    RateRoll = (float)gyro[0] / 131.0; // Scale to degrees per second
    RatePitch = (float)gyro[1] / 131.0;
    RateYaw = (float)gyro[2] / 131.0;

    printf("Acc. X = %f g, Acc. Y = %f g, Acc. Z = %f g\n", AccX, AccY, AccZ);
    printf("Rate Roll = %f deg/s, Rate Pitch = %f deg/s, Rate Yaw = %f deg/s\n", RateRoll, RatePitch, RateYaw);

    // Temperature is simple so use the datasheet calculation to get deg C.
    // Note this is chip temperature.
    printf("Temp. = %f\n", (*temp / 340.0) + 36.53);

}

void who_am_i_mpu6050() {
    
    // Read the device ID to check we are talking to the right device
    uint8_t buffer[1];
    uint8_t val = 0x75; // Who am I register
    i2c_write_blocking(i2c_default, addr_mpu, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, addr_mpu, buffer, 1, false); // False - finished with bus
    printf("Device ID = 0x%02X\n", buffer[0]);
}

void who_am_i_lsm303d() {
    // Read the device ID to check we are talking to the right device
    uint8_t buffer[1];
    uint8_t val = 0x0F; // Who am I register for LSM303D
    i2c_write_blocking(i2c_default, addr_LSM303D, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, addr_LSM303D, buffer, 1, false); // False - finished with bus
    printf("LSM303D Device ID = 0x%02X\n", buffer[0]);
}

static void setup_lsm303d() {
    // Reset the LSM303D
    uint8_t buf[] = {0x1F, 0x00}; // CTRL0 register, reset
    i2c_write_blocking(i2c_default, addr_LSM303D, buf, 2, false);
    
    buf[0] = 0x20; // CTRL1 register
    buf[1] = 0x47; // CTRL1 register, BDU = 0, set to 100Hz, all axes enabled
    i2c_write_blocking(i2c_default, addr_LSM303D, buf, 2, false);
    
    


    buf[0] = 0x21; // CTRL2 register
    buf[1] = 0x00; // Interrupts disabled
    i2c_write_blocking(i2c_default, addr_LSM303D, buf, 2, false);

    buf[0] = 0x24; // CTRL4
    buf[1] = 0x00; // Interrupts and FIFO disabled
    i2c_write_blocking(i2c_default, addr_LSM303D, buf, 2, false);


    
    who_am_i_lsm303d(); // Check we are talking to the right device
}

static void lsm303d_read_raw(int16_t accel[3], int16_t mag[3]) {
    uint8_t buffer[6];

    // Read acceleration data from register 0x28 for 6 bytes
    uint8_t val = 0x28 | 0x80; // Auto increment bit set
    i2c_write_blocking(i2c_default, addr_LSM303D, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, addr_LSM303D, buffer, 6, false);

    // for (int i = 0; i < 3; i++) {
    //     accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    // }

    accel[0] = (buffer[1] << 8 | buffer[0]);
    accel[1] = (buffer[3] << 8 | buffer[2]);
    accel[2] = (buffer[5] << 8 | buffer[4]);

    printf("Accel X = %d, Y = %d, Z = %d\n", accel[0], accel[1], accel[2]);

    // Scale dat to g's
    AccX = (float)accel[0] / 16384.0; // Scale to g's
    AccY = (float)accel[1] / 16384.0;
    AccZ = (float)accel[2] / 16384.0;

    printf("Scaled Acc. X = %f g, Acc. Y = %f g, Acc. Z = %f g\n", AccX, AccY, AccZ);
}

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

int main() {
    stdio_init_all();
    sleep_ms(2000); // Wait for device to power up

    printf("Hello, MPU6050! Reading raw data from registers...\n");

    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    // i2c_init(i2c_default, 400 * 1000);
    // gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    // gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    // gpio_pull_up(I2C_SDA);
    // gpio_pull_up(I2C_SCL);
    
    lsm303D_data_t lsm303d_data; // Create a data structure for LSM303D
    lsm303d_init(I2C_PORT, I2C_SDA, I2C_SCL, &lsm303d_data); // Initialize LSM303D

    setup_l3gd20(); // Set up the L3GD20 gyroscope

    // mpu6050_data_t mpu6050_data; // Create a data structure for MPU6050
    // mpu6050_init(I2C_PORT, I2C_SDA, I2C_SCL, &mpu6050_data); // Initialize MPU6050
    int16_t acceleration[3], gyro[3], temp;
    printf("aX:,aY:,aZ:,rX:,rY:,rZ:\n");
    while (1) {
        //mpu6050_read_raw(acceleration, gyro, &temp);
        // lsm303d_read_raw(acceleration, gyro);

        l3gd20_read_raw(gyro);

        lsm303d_read(&lsm303d_data); // Read data from LSM303D
        printf("LSM303D Accel X = %f g, Y = %f g, Z = %f g\n", 
               lsm303d_data.AccX, lsm303d_data.AccY, lsm303d_data.AccZ);

        // mpu6050_read(&mpu6050_data); // Read data from MPU6050
        // printf("%f,%f,%f,", 
        //        mpu6050_data.AccX, mpu6050_data.AccY, mpu6050_data.AccZ);
        // printf("%f,%f,%f\n", 
        //        mpu6050_data.RateRoll, mpu6050_data.RatePitch, mpu6050_data.RateYaw);
        sleep_ms(500);
    }
}
