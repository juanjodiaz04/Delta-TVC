#include "ssd1306.h"
#include "stdio.h"
#include "../mpu6050/mpu6050.h"

void screen_1(ssd1306_t *oled);
void screen_2(ssd1306_t *oled);
void screen_IMU_update(ssd1306_t *oled, mpu6050_data_t *mpu6050_data, float angle_roll_offset, float angle_pitch_offset, float current);
void screen_3(ssd1306_t *oled, const char *message);
void screen_4(ssd1306_t *oled, const char *param_name, float value);
void screen_initial_float_conversion(ssd1306_t *oled, const char *prompt);
void screen_update_float_conversion(ssd1306_t *oled, const char *prompt, const char *input_buffer, char *display_message);
void screen_confirmation(ssd1306_t *oled, const char *message, float result, char *display_message);
void screen_params_summary(ssd1306_t *oled, float ki, float kp, float kd, float setpoint, float speed);
void screen_buffer_error(ssd1306_t *oled);
void screen_decimal_exists_error(ssd1306_t *oled);
void screen_decimal_before_number_error(ssd1306_t *oled);
void screen_number_too_long_error(ssd1306_t *oled);
void screen_reset_input(ssd1306_t *oled, const char *prompt);
void screen_invalid_char_error(ssd1306_t *oled, const char *prompt, const char *input_buffer, char *display_message);
