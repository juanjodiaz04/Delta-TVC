#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#include "lib/LSM303D/lsm303D.h"
#include "lib/mpu6050/mpu6050.h"

#include "lib/servo_lib/servo_lib.h"
#include "lib/pid_lib/pid_lib.h"


int main()
{
    stdio_init_all();

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}
