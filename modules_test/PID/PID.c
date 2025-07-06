#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "lib/servo_lib/servo_lib.h"

int main() {
    stdio_init_all();
    servo_t my_servo;

    // Inicializar el servo en GPIO 15
    servo_init(&my_servo, 15);
    servo_start(&my_servo, 90);  // Posición inicial al centro

    printf("Listo. Escribe un ángulo entre 0 y 180 grados y presiona Enter:\n");

    char buffer[16];
    int index = 0;

    while (true) {
        int c = getchar_timeout_us(0);  // Lectura no bloqueante

        if (c != PICO_ERROR_TIMEOUT) {
            if (c == '\n' || c == '\r') {
                buffer[index] = '\0';  // Terminar cadena

                int angle = atoi(buffer);  // Convertir a entero

                if (angle >= 60 && angle <= 180) {
                    servo_set_angle(&my_servo, (uint8_t)angle);
                    printf("Ángulo aplicado: %d°\n", angle);
                } else {
                    printf("Ángulo inválido: %d. Debe estar entre 0 y 180.\n", angle);
                }

                index = 0;  // Reiniciar buffer
                printf("> ");
            } else if (index < sizeof(buffer) - 1) {
                buffer[index++] = (char)c;
            }
        }

        tight_loop_contents();  // Mantener eficiencia energética
    }

    return 0;
}
