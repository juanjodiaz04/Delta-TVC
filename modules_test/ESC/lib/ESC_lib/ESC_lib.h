#ifndef ESC_LIB_H
#define ESC_LIB_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

/**
 * @brief Estructura para el control básico de un ESC de motor brushless usando PWM.
 */
typedef struct {
    struct {
        uint8_t gpio_pin     : 5;   /**< Pin GPIO utilizado para la señal PWM. */
        uint8_t pwm_slice    : 3;   /**< Número de slice de PWM. */
        uint8_t pwm_channel  : 1;   /**< Canal PWM (0 o 1) dentro del slice. */

        uint16_t frequency_hz;      /**< Frecuencia PWM en Hz. */
        uint8_t duty_percent;       /**< Pulso actual en microsegundos. */
        uint16_t pwm_wrap;          /**< Valor máximo del contador PWM. */
        float clkdiv;               /**< Divisor de reloj para configurar frecuencia PWM. */

        bool armed;                 /**< Indica si el ESC está armado. */
    } BITS;
} esc_t;

/**
 * @brief Inicializa un ESC con los parámetros proporcionados.
 * 
 * @param esc Puntero a la estructura ESC.
 * @param gpio_pin Número de pin GPIO usado para la señal PWM.
 * @param frequency_hz Frecuencia PWM en Hz (típicamente 50 Hz).
 * @param clkdiv Divisor de reloj para el PWM.
 * @param min_pulse_us Pulso mínimo en microsegundos.
 * @param max_pulse_us Pulso máximo en microsegundos.
 */
void esc_init(esc_t* esc, uint8_t gpio_pin, uint16_t frequency_hz, float clkdiv);

/**
 * @brief Envía un pulso en microsegundos al ESC.
 * 
 * @param esc Puntero a la estructura ESC.
 */
void esc_write_duty(esc_t* esc, uint8_t duty_percent);

#endif // ESC_LIB_H
