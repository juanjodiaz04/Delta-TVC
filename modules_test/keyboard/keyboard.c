#include <stdio.h>
#include "pico/stdlib.h"
#include "lib/mat.h"

int main()
{
    stdio_init_all();
    init_mat();

    while (true) {
        uint8_t key=0;
        while (!read_mat(&key));

        printf("Key pressed: %c \n",key);
        sleep_ms(1000);
    }
}

