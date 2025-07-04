/**
 * @file servo.h
 * @brief Header file for RC servo control on Raspberry Pi Pico using PWM.
 *
 * This interface allows low-level control of hobby servos using
 * hardware PWM. It supports initialization, setting angles, and setting
 * direct pulse widths in microseconds.
 */

#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @struct servo_t
 * @brief Structure representing a servo PWM configuration.
 *
 * Contains the PWM slice and GPIO pin used to drive the servo.
 */
typedef struct {
    uint8_t pin;         /**< GPIO pin connected to the servo signal */
    uint8_t slice_num;   /**< PWM slice number associated with the pin */
    uint32_t top;       /**< PWM TOP value for the configured frequency */
    uint16_t angle;     /**< Current angle of the servo in degrees (0-180) */
} servo_t;

/**
 * @brief Initializes a GPIO pin for PWM-based servo control.
 *
 * Configures the PWM to 50 Hz and prepares the struct.
 *
 * @param servo Pointer to a servo_t structure to configure.
 * @param pin GPIO pin to be used for the servo signal.
 */
void servo_init(servo_t *servo, uint8_t pin, uint16_t initial_angle);

/**
 * @brief Starts PWM signal and sets servo to an initial angle.
 *
 * @param servo Pointer to a previously initialized servo_t struct.
 * @param angle Angle in degrees (0 to 180).
 */
void servo_start(const servo_t *servo, uint8_t angle);

/**
 * @brief Updates the PWM signal to move the servo to a new angle.
 *
 * @param servo Pointer to a servo_t struct.
 * @param angle Angle in degrees (0 to 180).
 */
void servo_set_angle(const servo_t *servo, uint8_t angle);

/**
 * @brief Sets a raw pulse width in microseconds for the servo.
 *
 * Useful for precise control or PID integration.
 *
 * @param servo Pointer to a servo_t struct.
 * @param micros Pulse width in µs (valid range: 1000 to 2000).
 */
void servo_set_micros(const servo_t *servo, uint16_t micros);

#endif // SERVO_H
