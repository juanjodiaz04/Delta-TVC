/**
 * @file ESC_lib.h
 * @brief Interfaz para controlar ESCs (Electronic Speed Controllers) usando PWM en el RP2040.
 *
 * Este archivo define la estructura y funciones necesarias para inicializar y controlar un ESC.
 * Se utiliza PWM para enviar señales de control al ESC, permitiendo ajustar la velocidad del motor.
 */

#ifndef ESC_LIB_H
#define ESC_LIB_H

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

typedef struct {
    struct {
        uint8_t gpio_pin     : 5;   /**< Pin GPIO utilizado para la señal PWM. */
        uint8_t pwm_slice    : 3;   /**< Número de slice de PWM. */
        uint8_t pwm_channel  : 1;   /**< Canal PWM (0 o 1) dentro del slice. */

        uint16_t frequency_hz;      /**< Frecuencia PWM en Hz. */
        float duty_percent;         /**< Porcentaje de duty cycle actual del PWM. */
        uint8_t speed_percent;       /**< Porcentaje de velocidad actual del motor. */
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
void esc_write_speed(esc_t* esc, uint8_t speed_percent);

#endif // ESC_LIB_H
