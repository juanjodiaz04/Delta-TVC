#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "ESC_lib.h"

/**
 * @brief Inicializa un ESC con los parámetros proporcionados.
 */
void esc_init(esc_t *esc, uint8_t gpio_pin, uint16_t frequency_hz, float clkdiv)
{
    // Guardar parámetros en la estructura
    esc->BITS.gpio_pin = gpio_pin;
    esc->BITS.frequency_hz = frequency_hz;
    esc->BITS.clkdiv = clkdiv;

    // Configurar GPIO como salida PWM
    gpio_init(gpio_pin);
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);

    // Obtener slice y canal asociados al GPIO
    esc->BITS.pwm_slice = pwm_gpio_to_slice_num(gpio_pin);
    esc->BITS.pwm_channel = pwm_gpio_to_channel(gpio_pin);

    // Calcular wrap
    uint32_t sys_clk = 125000000;
    esc->BITS.pwm_wrap = (uint16_t)((sys_clk / (frequency_hz * clkdiv)) - 1);

    // Configurar PWM
    pwm_set_clkdiv(esc->BITS.pwm_slice, clkdiv);
    pwm_set_wrap(esc->BITS.pwm_slice, esc->BITS.pwm_wrap);
    pwm_set_chan_level(esc->BITS.pwm_slice, esc->BITS.pwm_channel, 0);
    pwm_set_enabled(esc->BITS.pwm_slice, true);

    esc->BITS.duty_percent = 0;
    esc->BITS.armed = false;
}

void esc_write_duty(esc_t *esc, uint8_t duty_percent)
{
    if (duty_percent > 100) duty_percent = 100; // Limitar a 100%

    // Calcular el valor del nivel PWM basado en porcentaje
    uint16_t level = ((esc->BITS.pwm_wrap + 1) * duty_percent) / 100;

    // Aplicar el valor al canal correspondiente
    pwm_set_chan_level(esc->BITS.pwm_slice, esc->BITS.pwm_channel, level);

}
