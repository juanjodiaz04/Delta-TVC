/**
 * @file servo.c
 * @brief Minimal RC servo control using PWM on Raspberry Pi Pico.
 *
 * This file provides basic functions to control hobby servos via
 * GPIO and hardware PWM. Servos are controlled using pulses from
 * 1000 to 2000 µs at a frequency of 50 Hz.
 */

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "lib/servo_lib/servo_lib.h"

#define SERVO_MIN_PULSE_US  1000   /**< Minimum pulse width in µs */
#define SERVO_MAX_PULSE_US  2000   /**< Maximum pulse width in µs */
#define SERVO_MIN_ANGLE     0      /**< Minimum angle in degrees */
#define SERVO_MAX_ANGLE     180    /**< Maximum angle in degrees */
#define SERVO_PERIOD_US     20000  /**< PWM period for 50 Hz (20 ms) */

/**
 * @brief Maps a value from one range to another.
 */
static inline uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max,
                           uint16_t out_min, uint16_t out_max)
{
    return (uint16_t)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

/**
 * @brief Converts an angle in degrees to a pulse width in microseconds.
 */
static uint16_t angle_to_micros(uint8_t angle)
{
    return map(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
}

void servo_init(servo_t *servo, uint8_t pin)
{
    servo->pin = pin;
    servo->slice_num = pwm_gpio_to_slice_num(pin);

    gpio_set_function(pin, GPIO_FUNC_PWM);

    servo->clkdiv = 64.0f;
    servo->freq = 50;
    uint32_t top = (uint32_t)((clock_get_hz(clk_sys) / (servo->clkdiv * servo->freq)) - 1);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, servo->clkdiv);
    pwm_config_set_wrap(&config, top);

    pwm_init(servo->slice_num, &config, false);
}

void servo_start(const servo_t *servo, uint8_t angle)
{
    if (angle < SERVO_MIN_ANGLE || angle > SERVO_MAX_ANGLE)
        return;

    uint16_t pulse = angle_to_micros(angle);
    uint8_t channel = pwm_gpio_to_channel(servo->pin);

    pwm_set_chan_level(servo->slice_num, channel, pulse);
    pwm_set_enabled(servo->slice_num, true);
}

void servo_set_angle(const servo_t *servo, uint8_t angle)
{
    if (angle < SERVO_MIN_ANGLE || angle > SERVO_MAX_ANGLE)
        return;

    uint16_t pulse = angle_to_micros(angle);
    uint8_t channel = pwm_gpio_to_channel(servo->pin);

    pwm_set_chan_level(servo->slice_num, channel, pulse);
}

void servo_set_micros(const servo_t *servo, uint16_t micros)
{
    if (micros < SERVO_MIN_PULSE_US || micros > SERVO_MAX_PULSE_US)
        return;

    uint8_t channel = pwm_gpio_to_channel(servo->pin);
    pwm_set_chan_level(servo->slice_num, channel, micros);
}

void servo_set_freq_and_clkdiv(servo_t *servo, uint16_t freq, float clkdiv)
{
    if (freq == 0 || clkdiv == 0.0f)
        return;

    servo->freq = freq;
    servo->clkdiv = clkdiv;

    uint32_t top = (uint32_t)((clock_get_hz(clk_sys) / (servo->clkdiv * servo->freq)) - 1);
    pwm_set_wrap(servo->slice_num, top);
}
