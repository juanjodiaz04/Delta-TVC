/**
 * @file servo.c
 * @brief Minimal RC servo control using PWM on Raspberry Pi Pico.
 *
 * This file provides basic functions to control hobby servos via
 * GPIO and hardware PWM. Servos are controlled using pulses from
 * 1000 to 2000 µs at a frequency of 50 Hz and a fixed clkdiv of 64.
 */

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "lib/servo_lib/servo_lib.h"

#define SERVO_MIN_PULSE_US  600   /**< Minimum pulse width in µs */
#define SERVO_MAX_PULSE_US  2400   /**< Maximum pulse width in µs */
#define SERVO_MIN_ANGLE     0      /**< Minimum angle in degrees */
#define SERVO_MAX_ANGLE     180    /**< Maximum angle in degrees */
#define SERVO_PERIOD_US     20000  /**< 50 Hz = 20 ms period */
#define SERVO_CLKDIV        64.0f  /**< Fixed PWM clock divider */

/**
 * @brief Maps a value from one range to another.
 */
static inline uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max,
                           uint16_t out_min, uint16_t out_max)
{
    return (uint16_t)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

/**
 * @brief Converts an angle to the corresponding PWM level based on configured TOP.
 */
static uint16_t angle_to_level(const servo_t *servo, uint8_t angle)
{
    uint16_t micros = map(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE,
                                 SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
    return (uint16_t)((micros * servo->top) / SERVO_PERIOD_US);
}

/**
 * @brief Converts microseconds to PWM level based on configured TOP.
 */
static uint16_t micros_to_level(const servo_t *servo, uint16_t micros)
{
    return (uint16_t)((micros * servo->top) / SERVO_PERIOD_US);
}

void servo_init(servo_t *servo, uint8_t pin, uint16_t initial_angle)
{
    servo->pin = pin;
    servo->slice_num = pwm_gpio_to_slice_num(pin);

    gpio_set_function(pin, GPIO_FUNC_PWM);

    uint32_t top = (uint32_t)((clock_get_hz(clk_sys) / (SERVO_CLKDIV * 50)) - 1);
    servo->top = top;

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, SERVO_CLKDIV);
    pwm_config_set_wrap(&config, top);

    pwm_init(servo->slice_num, &config, false);
    pwm_set_gpio_level(pin, angle_to_level(servo, initial_angle));
    servo->angle = initial_angle;
}

void servo_start(servo_t *servo, uint8_t angle)
{
    if (angle < SERVO_MIN_ANGLE || angle > SERVO_MAX_ANGLE)
        return;

    uint16_t level = angle_to_level(servo, angle);
    uint8_t channel = pwm_gpio_to_channel(servo->pin);

    pwm_set_chan_level(servo->slice_num, channel, level);
    pwm_set_enabled(servo->slice_num, true);
}

void servo_set_angle(servo_t *servo, uint8_t angle)
{
    if (angle < SERVO_MIN_ANGLE || angle > SERVO_MAX_ANGLE)
        return;

    uint16_t level = angle_to_level(servo, angle);
    uint8_t channel = pwm_gpio_to_channel(servo->pin);
    servo->angle = angle;
    pwm_set_chan_level(servo->slice_num, channel, level);
}

void servo_set_micros(const servo_t *servo, uint16_t micros)
{
    if (micros < SERVO_MIN_PULSE_US || micros > SERVO_MAX_PULSE_US)
        return;

    uint16_t level = micros_to_level(servo, micros);
    uint8_t channel = pwm_gpio_to_channel(servo->pin);

    pwm_set_chan_level(servo->slice_num, channel, level);
}