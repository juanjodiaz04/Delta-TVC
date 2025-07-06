/**
 * @file screen.h
 * @brief OLED screen interface for displaying various application screens.
 *
 * This library provides a set of functions to render user interface screens
 * on an SSD1306 OLED display, including menus, sensor data (IMU readings),
 * parameter adjustment prompts, confirmation messages, and error notifications.
 *
 * It is designed for embedded applications such as control systems using the
 * MPU6050 IMU and a small OLED display.
 */

#ifndef SCREEN_H
#define SCREEN_H

#include "ssd1306.h"
#include "../mpu6050/mpu6050.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Display the main menu screen.
 *
 * Shows the initial screen with options to select automatic or manual mode.
 *
 * @param oled Pointer to initialized SSD1306 display context.
 */
void screen_1(ssd1306_t *oled);

/**
 * @brief Display the automatic mode screen with sensor labels.
 *
 * Prepares the display area to show IMU acceleration and angle data.
 *
 * @param oled Pointer to initialized SSD1306 display context.
 */
void screen_2(ssd1306_t *oled);

/**
 * @brief Update the automatic mode screen with IMU sensor data and current.
 *
 * Overwrites the sensor values area on the screen with IMU and current measurements.
 *
 * @param oled Pointer to SSD1306 display context.
 * @param mpu6050_data Pointer to MPU6050 data structure containing sensor readings.
 * @param angle_roll_offset Offset to apply to roll angle.
 * @param angle_pitch_offset Offset to apply to pitch angle.
 * @param current Current measurement (Amperes).
 */
void screen_IMU_update(ssd1306_t *oled, mpu6050_data_t *mpu6050_data, float angle_roll_offset, float angle_pitch_offset, float current);

/**
 * @brief Display the manual mode screen with a custom message.
 *
 * @param oled Pointer to SSD1306 display context.
 * @param message Message to display below the "Manual Mode" header.
 */
void screen_3(ssd1306_t *oled, const char *message);

/**
 * @brief Show a screen displaying a parameter name and its float value.
 *
 * @param oled Pointer to SSD1306 display context.
 * @param param_name Name of the parameter.
 * @param value Value assigned to the parameter.
 */
void screen_4(ssd1306_t *oled, const char *param_name, float value);

/**
 * @brief Show initial prompt for entering a float value.
 *
 * This screen shows the instructions for entering a float using keypad.
 *
 * @param oled Pointer to SSD1306 display context.
 * @param prompt Instruction text to display.
 */
void screen_initial_float_conversion(ssd1306_t *oled, const char *prompt);

/**
 * @brief Update the float input screen with the current input buffer.
 *
 * @param oled Pointer to SSD1306 display context.
 * @param prompt Instruction text.
 * @param input_buffer String containing the current user input.
 * @param display_message Buffer to prepare the display text (must be at least 64 bytes).
 */
void screen_update_float_conversion(ssd1306_t *oled, const char *prompt, const char *input_buffer, char *display_message);

/**
 * @brief Show confirmation screen after entering a float value.
 *
 * @param oled Pointer to SSD1306 display context.
 * @param message Confirmation message.
 * @param result Confirmed float value.
 * @param display_message Buffer to prepare the display text (must be at least 64 bytes).
 */
void screen_confirmation(ssd1306_t *oled, const char *message, float result, char *display_message);

/**
 * @brief Show a summary of PID parameters and settings before confirmation.
 *
 * @param oled Pointer to SSD1306 display context.
 * @param ki Integral gain.
 * @param kp Proportional gain.
 * @param kd Derivative gain.
 * @param setpoint Target setpoint value.
 * @param speed ESC speed setting.
 */
void screen_params_summary(ssd1306_t *oled, float ki, float kp, float kd, float setpoint, float speed);

/**
 * @brief Show an error screen indicating that the input buffer exceeded its length.
 *
 * @param oled Pointer to SSD1306 display context.
 */
void screen_buffer_error(ssd1306_t *oled);

/**
 * @brief Show an error screen indicating that a decimal point already exists.
 *
 * @param oled Pointer to SSD1306 display context.
 */
void screen_decimal_exists_error(ssd1306_t *oled);

/**
 * @brief Show an error screen indicating that the decimal point was inserted before any number.
 *
 * @param oled Pointer to SSD1306 display context.
 */
void screen_decimal_before_number_error(ssd1306_t *oled);

/**
 * @brief Show an error screen indicating that the entered number is too long.
 *
 * @param oled Pointer to SSD1306 display context.
 */
void screen_number_too_long_error(ssd1306_t *oled);

/**
 * @brief Show a screen indicating that input has been cleared.
 *
 * @param oled Pointer to SSD1306 display context.
 * @param prompt Optional prompt or message to display.
 */
void screen_reset_input(ssd1306_t *oled, const char *prompt);

/**
 * @brief Show an error screen indicating that the user entered an invalid character.
 *
 * @param oled Pointer to SSD1306 display context.
 * @param prompt Instruction text.
 * @param input_buffer Current input string.
 * @param display_message Buffer to prepare the display text (must be at least 64 bytes).
 */
void screen_invalid_char_error(ssd1306_t *oled, const char *prompt, const char *input_buffer, char *display_message);

#ifdef __cplusplus
}
#endif

#endif // SCREEN_H