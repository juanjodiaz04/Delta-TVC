#include "screen.h"


/**
 * TODO: This functions should be in the oled library. 
 */

void screen_1(ssd1306_t *oled)
{
    ssd1306_clear(oled);
    ssd1306_draw_string(oled, 0, 0, 1, "TVC Control");
    ssd1306_draw_string(oled, 0, 10, 1, "Select option:");
    ssd1306_draw_string(oled, 0, 20, 1, "1. Automatic");
    ssd1306_draw_string(oled, 0, 30, 1, "2. Manual");
    ssd1306_show(oled);
}

void screen_2(ssd1306_t *oled){
    ssd1306_clear(oled);
    ssd1306_draw_string(oled, 0, 0, 1, "Automatic Mode");
    ssd1306_draw_string(oled, 0, 20, 1, "Ac_x = 0.00");
    ssd1306_draw_string(oled, 0, 30, 1, "Ac_y = 0.00");
    ssd1306_draw_string(oled, 0, 45, 1, "Ag_x = 0.00");
    ssd1306_draw_string(oled, 0, 55, 1, "Ag_y = 0.00");
    ssd1306_show(oled);
}

void screen_IMU_update(ssd1306_t *oled, mpu6050_data_t *mpu6050_data, float angle_roll_offset, float angle_pitch_offset){

    char buffer[32];
    ssd1306_clear_square(oled, 35, 10, 64, 54); // Clear the area for new data
    snprintf(buffer, sizeof(buffer), " %.2f", mpu6050_data->AccX);
    ssd1306_draw_string(oled, 35, 20, 1, buffer);
    snprintf(buffer, sizeof(buffer), " %.2f", mpu6050_data->AccY);
    ssd1306_draw_string(oled, 35, 30, 1, buffer);

    //Angulos
    snprintf(buffer, sizeof(buffer), " %.2f", mpu6050_data->KalmanAngleRoll + angle_roll_offset);
    ssd1306_draw_string(oled, 35, 45, 1, buffer);
    snprintf(buffer, sizeof(buffer), " %.2f", mpu6050_data->KalmanAnglePitch + angle_pitch_offset);
    ssd1306_draw_string(oled, 35, 55, 1, buffer);
    ssd1306_show(oled);
}

void screen_3(ssd1306_t *oled, const char *message){
    ssd1306_clear(oled);
    ssd1306_draw_string(oled, 0, 0, 1, "Manual Mode");
    ssd1306_draw_string(oled, 0, 10, 1, message);
    ssd1306_show(oled);
}

void screen_4(ssd1306_t *oled, const char *param_name, float value){
    ssd1306_clear(oled);
    ssd1306_draw_string(oled, 0, 0, 1, "Parameter Set");
    char param_line[32];
    snprintf(param_line, sizeof(param_line), "%s = %.3f", param_name, value);
    ssd1306_draw_string(oled, 0, 20, 1, param_line);
    ssd1306_draw_string(oled, 0, 40, 1, "Press any key");
    ssd1306_draw_string(oled, 0, 50, 1, "to continue...");
    ssd1306_show(oled);
}


void screen_initial_float_conversion(ssd1306_t *oled, const char *prompt) {
    ssd1306_clear(oled);
    ssd1306_draw_string(oled, 0, 0, 1, prompt);
    ssd1306_draw_string(oled, 0, 20, 1, "Enter number:");
    ssd1306_draw_string(oled, 0, 30, 1, "* = decimal point");
    ssd1306_draw_string(oled, 0, 40, 1, "# = confirm");
    ssd1306_draw_string(oled, 0, 50, 1, "Value: ");
    ssd1306_show(oled);
}

void screen_update_float_conversion(ssd1306_t *oled, const char *prompt, const char *input_buffer, char *display_message) {
    ssd1306_clear(oled);
    ssd1306_draw_string(oled, 0, 0, 1, prompt);
    ssd1306_draw_string(oled, 0, 20, 1, "Enter number:");
    ssd1306_draw_string(oled, 0, 30, 1, "* = decimal point");
    ssd1306_draw_string(oled, 0, 40, 1, "# = confirm");
    snprintf(display_message, 64, "Value: %s", input_buffer);
    ssd1306_draw_string(oled, 0, 50, 1, display_message);
    ssd1306_show(oled);
}

void screen_confirmation(ssd1306_t *oled, const char *message, float result, char *display_message) {
    ssd1306_clear(oled);
    ssd1306_draw_string(oled, 0, 0, 1, "Confirmed:");
    snprintf(display_message, 64, "%.3f", result);
    ssd1306_draw_string(oled, 0, 20, 1, display_message);
    ssd1306_show(oled);
}

void screen_params_summary(ssd1306_t *oled, float ki, float kp, float kd, float setpoint, float speed) {
    ssd1306_clear(oled);
    ssd1306_draw_string(oled, 0, 0, 1, "Parameters Set:");
    char summary[64];
    snprintf(summary, sizeof(summary), "Ki=%.2f Kp=%.2f", ki, kp);
    ssd1306_draw_string(oled, 0, 15, 1, summary);
    snprintf(summary, sizeof(summary), "Kd=%.2f SP=%.2f", kd, setpoint);
    ssd1306_draw_string(oled, 0, 25, 1, summary);

    snprintf(summary, sizeof(summary), "ESC Speed=%d", speed);
    ssd1306_draw_string(oled, 0, 35, 1, summary);
    ssd1306_draw_string(oled, 0, 45, 1, "Press # to confirm");
    ssd1306_draw_string(oled, 0, 55, 1, "Press * to reset");
    ssd1306_show(oled);
}

void screen_buffer_error(ssd1306_t *oled) {
    ssd1306_clear(oled);
    ssd1306_draw_string(oled, 0, 0, 1, "ERROR:");
    ssd1306_draw_string(oled, 0, 10, 1, "Number too long");
    ssd1306_draw_string(oled, 0, 20, 1, "Press # to reset");
    ssd1306_show(oled);
}

void screen_decimal_exists_error(ssd1306_t *oled) {
    ssd1306_clear(oled);
    ssd1306_draw_string(oled, 0, 0, 1, "ERROR:");
    ssd1306_draw_string(oled, 0, 10, 1, "Decimal already");
    ssd1306_draw_string(oled, 0, 20, 1, "exists");
    ssd1306_draw_string(oled, 0, 40, 1, "Continue typing...");
    ssd1306_show(oled);
}

void screen_decimal_before_number_error(ssd1306_t *oled) {
    ssd1306_clear(oled);
    ssd1306_draw_string(oled, 0, 0, 1, "ERROR:");
    ssd1306_draw_string(oled, 0, 10, 1, "Enter number");
    ssd1306_draw_string(oled, 0, 20, 1, "before decimal");
    ssd1306_draw_string(oled, 0, 40, 1, "Continue typing...");
    ssd1306_show(oled);
}

void screen_number_too_long_error(ssd1306_t *oled) {
    ssd1306_clear(oled);
    ssd1306_draw_string(oled, 0, 0, 1, "ERROR:");
    ssd1306_draw_string(oled, 0, 10, 1, "Number too long");
    ssd1306_draw_string(oled, 0, 20, 1, "Press # to reset");
    ssd1306_show(oled);
}

void screen_reset_input(ssd1306_t *oled, const char *prompt) {
    ssd1306_clear(oled);
    ssd1306_draw_string(oled, 0, 0, 1, "Input cleared");
    ssd1306_draw_string(oled, 0, 20, 1, "Start over...");
    ssd1306_show(oled);

    sleep_ms(1000);  // Show reset message for 1 second
    
    // Redraw initial screen
    screen_initial_float_conversion(oled, prompt);
}

void screen_invalid_char_error(ssd1306_t *oled, const char *prompt, const char *input_buffer, char *display_message) {
    ssd1306_clear(oled);
    ssd1306_draw_string(oled, 0, 0, 1, "ERROR:");
    ssd1306_draw_string(oled, 0, 10, 1, "Invalid character");
    ssd1306_draw_string(oled, 0, 20, 1, "Use numbers only");
    ssd1306_draw_string(oled, 0, 40, 1, "Continue typing...");
    ssd1306_show(oled);

    sleep_ms(2000);  // Show error for 2 seconds

    // Redraw current input
    screen_update_float_conversion(oled, prompt, input_buffer, display_message);

}
