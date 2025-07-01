#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "lib/PID_lib/PID_lib.h"
#include "lib/servo_lib/servo_lib.h"

int main() {
    stdio_init_all();
    servo_t my_servo;

    // Usa el pin GPIO 16 como ejemplo
    servo_init(&my_servo, 15);
    servo_start(&my_servo, 0);  // Comienza en 0 grados

    uint64_t last_time = time_us_64();
    uint8_t angle = -20;
    bool subiendo = true;
    bool terminado = false;

    while (!terminado) {
        uint64_t now = time_us_64();

        // Cambiar cada 2 segundos
        if (now - last_time >= 2000000) {
            last_time = now;

            if (subiendo) {
                angle += 20;
                if (angle >= 180) {
                    angle = 180;
                    subiendo = false;
                }
            } else {
                angle -= 20;
                if (angle <= 0) {
                    angle = 0;
                    terminado = true;
                }
            }

            servo_set_angle(&my_servo, angle);
            printf("Ángulo actual: %d°\n", angle);
        }

        tight_loop_contents();
    }

    printf("Test finalizado. Servo en 90°\n");

    // Mantener en 90°
    while (true) {
        tight_loop_contents();
    }

    return 0;
}

