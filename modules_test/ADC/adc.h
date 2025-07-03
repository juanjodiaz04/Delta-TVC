#ifndef ADC_H
#define ADC_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

/*
 * ADVERTENCIA IMPORTANTE:
 * Si el sensor ACS712 se alimenta con 5V, su salida puede superar 3.3V.
 * El ADC del RP2040 solo acepta hasta 3.3V en la entrada.
 * Conectar la salida directamente al ADC puede:
 *  - Saturar la medición (valores truncados a 4095)
 *  - Dañar el microcontrolador permanentemente
 *
 * Para evitar esto:
 *  - Use un divisor resistivo adecuado para reducir la señal máxima por debajo de 3.3V
 *  - O alimente el ACS712 con 3.3V (si el módulo lo soporta)
 *  - O utilice un circuito adaptador de nivel
 *
 * Cambiar el valor de vref en el software NO amplía el rango físico del ADC.
 */

/**
 * @brief Estructura para manejar la configuración del ADC.
 */
typedef struct {
    uint8_t adc_input;      /**< Canal de ADC (0 a 3) */
    uint8_t gpio_pin;       /**< Pin GPIO asociado al canal */
    float vref;             /**< Voltaje de referencia (usualmente 3.3 V) */
} adc_t;

/**
 * @brief Inicializa un canal ADC específico.
 * 
 * @param adc Puntero a la estructura adc_t.
 * @param gpio_pin Pin GPIO a usar.
 * @param adc_input Canal ADC correspondiente.
 * @param vref Voltaje de referencia (típicamente 3.3 V).
 */
void adc_util_init(adc_t* adc, uint8_t gpio_pin, uint8_t adc_input, float vref);

/**
 * @brief Lee el valor crudo del ADC (0 - 4095).
 * 
 * @param adc Puntero a la estructura adc_t.
 * @return uint16_t Valor ADC.
 */
uint16_t adc_util_read_raw(adc_t* adc);

/**
 * @brief Lee el voltaje convertido.
 * 
 * @param adc Puntero a la estructura adc_t.
 * @return float Voltaje medido.
 */
float adc_util_read_voltage(adc_t* adc);

/**
 * @brief Convierte la lectura ADC en corriente en amperios usando un sensor ACS712.
 * 
 * @param adc Puntero a la estructura adc_t.
 * @param v_offset Voltaje de referencia en reposo (usualmente 2.5 V).
 * @param sensitivity Sensibilidad en V/A (por ejemplo, 0.185 V/A para el ACS712-05A).
 * @return float Corriente medida en amperios.
 */
float adc_read_current_acs712(adc_t* adc, float v_offset, float sensitivity);



#endif // ADC_UTIL_H
