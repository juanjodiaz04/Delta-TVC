#include <stdio.h>
#include "pico/stdlib.h"
#include "lib/ESC_lib/ESC_lib.h"

int main() {
    // Inicializar el sistema estándar de entrada/salida
    stdio_init_all();

    // Crear e inicializar el ESC
    esc_t my_esc;
    esc_init(&my_esc, 15, 50, 64.0f );

    // Esperar a que el ESC esté listo (algunos requieren armarse)
    sleep_ms(2000);

    // Barrido de potencia de 0% a 100%
    for (uint8_t duty = 0; duty <= 100; duty += 10) {
        esc_write_duty(&my_esc, duty);
        printf("Duty cycle: %u%%\n", duty);
        sleep_ms(500);
    }

    // Mantener a 50%
    esc_write_duty(&my_esc, 50);
    printf("Duty cycle estable en 50%%\n");

    // Bucle infinito
    while (true) {
        tight_loop_contents();
    }

    return 0;
}