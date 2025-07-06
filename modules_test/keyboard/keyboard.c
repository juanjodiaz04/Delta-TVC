#include <stdio.h>
#include "pico/stdlib.h"
#include "lib/mat.h"
//callback de timer para enviar un mensaje diferente cada segundo
static repeating_timer_t timer_imu;
volatile uint8_t key = 0;

bool timer_callback(struct repeating_timer *t)
{
    static int count = 0;
    printf("Timer callback: %d\n", count++);
    return true; // Keep the timer running
}
int main()
{
    stdio_init_all();
    init_mat();

    // Initialize the timer to call timer_callback every second
    add_repeating_timer_ms(1000, timer_callback, NULL, &timer_imu);

    while (true) {
        
        if (get_key_flag()) {
            if (read_mat(&key)) {
                printf("Key pressed: %c\n", key);
            }
        }

        tight_loop_contents(); 
    }
}

