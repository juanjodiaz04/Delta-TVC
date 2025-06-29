#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "lib/ssd1306.h"
// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

ssd1306_t oled;


int main()
{
    stdio_init_all();
    sleep_ms(1000); // Wait for the serial console to be ready
    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    oled.external_vcc=false;
	bool res = ssd1306_init(
			&oled,
			128,
			64,
			0x3c,
			i2c0);
    if(!res)printf("OLED initialization failed!\n");
    ssd1306_clear(&oled);

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
        ssd1306_clear(&oled);
        ssd1306_draw_string(&oled, 0, 2, 1, "Hello, world!");
        ssd1306_show(&oled);

        sleep_ms(1000);

        ssd1306_clear(&oled);
        ssd1306_draw_string(&oled, 0, 20, 1, "Goodbye!");
        ssd1306_draw_line(
			&oled,
			2, 25,
			80, 25);
        ssd1306_show(&oled);
    }
}
