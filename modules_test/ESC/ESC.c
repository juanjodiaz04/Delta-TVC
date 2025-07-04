#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "lib/ESC_lib/ESC_lib.h"

int main() {
    stdio_init_all();

    esc_t my_esc;
    esc_init(&my_esc, 15, 50, 64.0f); // GPIO 15, 50 Hz, clkdiv 64
    esc_write_speed(&my_esc, 0);
    sleep_ms(2000);  // Tiempo para armar el ESC

    printf("Listo. Escribe un valor entre 0 y 100 y presiona Enter:\n");

    char buffer[16];
    int index = 0;

    while (true) {
        int c = getchar_timeout_us(0);  // Lectura no bloqueante

        if (c != PICO_ERROR_TIMEOUT) {
            if (c == '\r' || c == '\n') {
                buffer[index] = '\0';
                int speed = atoi(buffer);

                if (speed >= 0 && speed <= 100) {
                    esc_write_speed(&my_esc, (uint8_t)speed);
                    printf("Velocidad aplicada: %d%%\n", speed);
                } else {
                    printf("Valor inválido: %d. Debe ser entre 0 y 100.\n", speed);
                }

                index = 0;
                printf("> ");
            } else if (index < sizeof(buffer) - 1) {
                buffer[index++] = (char)c;
            }
        }

        tight_loop_contents();  // Bajo consumo
    }

    return 0;
}
