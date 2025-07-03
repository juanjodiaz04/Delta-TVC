#include <stdio.h>
#include "pico/stdlib.h"
#include "adc.h"

int main() {
    stdio_init_all();
    printf("Lectura de corriente con ACS712\n");

    adc_t adc_sensor;
    adc_util_init(&adc_sensor, 26, 0, 3.3f);

    // Configuración específica del ACS712
    const float V_OFFSET = 3.3f / 2.0f;     // Asumimos Vcc = 3.3 V
    const float SENSITIVITY = 0.185f;       // 185 mV/A para ACS712-05A

    while (1) {
        float current = adc_util_read_current_acs712(&adc_sensor, V_OFFSET, SENSITIVITY);
        printf("Corriente: %.3f A\n", current);
        sleep_ms(500);
    }
}
// This code initializes the ADC, reads the current using an ACS712 sensor, and prints the current value to the console.