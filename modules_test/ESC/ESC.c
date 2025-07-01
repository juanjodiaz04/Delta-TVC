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

    // Barrido de potencia no bloqueante
    uint8_t duty = 0;
    uint64_t last_update = time_us_64();
    bool barrido_completo = false;

    // Establecer duty inicial
    esc_write_duty(&my_esc, duty);
    printf("Duty cycle: %u%%\n", duty);

    while (true) {
        uint64_t now = time_us_64();

        if (!barrido_completo && now - last_update >= 2000000) {  // 2 segundos
            duty += 10;
            if (duty > 100) {
                duty = 50;
                barrido_completo = true;
                printf("Duty cycle estable en 50%%\n");
                esc_write_duty(&my_esc, duty);
            } else {
                esc_write_duty(&my_esc, duty);
                printf("Duty cycle: %u%%\n", duty);
            }
            last_update = now;
        }

    }

    return 0;
}
