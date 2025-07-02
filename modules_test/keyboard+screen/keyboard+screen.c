#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "lib/ssd1306.h"
#include "lib/mat.h"
#include "lib/I2C_driver/i2c_driver.h"
#include <string.h>  // Added for string functions
#include <stdlib.h>
// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 20
#define I2C_SCL 21

#define MAX_INPUT_LENGTH 16  // Maximum characters for number input

ssd1306_t oled;

float ki, kp, kd, setpoint; // PID variables

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
    ssd1306_draw_string(oled, 0, 10, 1, "Ac_x = 0.00");
    ssd1306_draw_string(oled, 0, 20, 1, "Ac_y = 0.00");
    ssd1306_draw_string(oled, 0, 30, 1, "Ac_z = 0.00");
    ssd1306_draw_string(oled, 0, 40, 1, "Gy_x = 0.00");
    ssd1306_draw_string(oled, 0, 50, 1, "Gy_y = 0.00");
    ssd1306_draw_string(oled, 0, 60, 1, "Gy_z = 0.00");
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

void screen_params_summary(ssd1306_t *oled, float ki, float kp, float kd, float setpoint) {
    ssd1306_clear(oled);
    ssd1306_draw_string(oled, 0, 0, 1, "Parameters Set:");
    char summary[64];
    snprintf(summary, sizeof(summary), "Ki=%.2f Kp=%.2f", ki, kp);
    ssd1306_draw_string(oled, 0, 15, 1, summary);
    snprintf(summary, sizeof(summary), "Kd=%.2f", kd);
    ssd1306_draw_string(oled, 0, 25, 1, summary);
    snprintf(summary, sizeof(summary), "SP=%.2f", setpoint);
    ssd1306_draw_string(oled, 0, 35, 1, summary);
    ssd1306_draw_string(oled, 0, 50, 1, "# = Main Menu");
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

/**
 * TODO: Change sleeps by timers or other technique.
 */


float keyboard_to_float(ssd1306_t *oled, const char *prompt) {
    char input_buffer[MAX_INPUT_LENGTH + 1] = {0};  // +1 for null terminator
    uint8_t buffer_index = 0;
    bool has_decimal = false;
    uint8_t key = 0;
    char display_message[64];
    
    // Initial screen display
    screen_initial_float_conversion(oled, prompt);
    
    while (true) {
        if (get_key_flag()) {
            while (!read_mat(&key));
            
            // Check if it's a number (0-9)
            if (key >= '0' && key <= '9') {
                // Check if buffer is not full
                if (buffer_index < MAX_INPUT_LENGTH) {
                    input_buffer[buffer_index] = key;
                    buffer_index++;
                    input_buffer[buffer_index] = '\0';  // Null terminate
                    
                    // Update screen
                    screen_update_float_conversion(oled, prompt, input_buffer, display_message);
                    
                } else {
                    // Buffer full, show error
                    screen_buffer_error(oled);
                }
            }
            // Check if it's decimal point (*)
            else if (key == '*') {
                // Only allow one decimal point
                if (!has_decimal && buffer_index > 0 && buffer_index < MAX_INPUT_LENGTH) {
                    input_buffer[buffer_index] = '.';
                    buffer_index++;
                    input_buffer[buffer_index] = '\0';
                    has_decimal = true;
                    
                    // Update screen
                    screen_update_float_conversion(oled, prompt, input_buffer, display_message);
                    
                } else {
                    // Invalid decimal point usage
                    if (has_decimal) {
                        screen_decimal_exists_error(oled);
                    } else if (buffer_index == 0) {
                        screen_decimal_before_number_error(oled);
                    } else {
                        screen_number_too_long_error(oled);}
                    
                    sleep_ms(2000);  // Show error for 2 seconds
                    
                    // Redraw current input
                    screen_update_float_conversion(oled, prompt, input_buffer, display_message);
                }
            }
            // Check if it's confirm (#)
            else if (key == '#') {
                if (buffer_index > 0) {
                    // Check if the last character is a decimal point
                    if (input_buffer[buffer_index - 1] == '.') {
                        // Remove trailing decimal point
                        input_buffer[buffer_index - 1] = '\0';
                        buffer_index--;
                        has_decimal = false;
                    }
                    
                    // Convert to float
                    float result = atof(input_buffer);
                    
                    // Show confirmation
                    screen_confirmation(oled, prompt, result, display_message);
                    sleep_ms(1500);  // Show confirmation for 1.5 seconds
                    
                    return result;
                } else {
                    // Empty input, reset
                    memset(input_buffer, 0, sizeof(input_buffer));
                    buffer_index = 0;
                    has_decimal = false;
                    
                    // Show reset message
                    screen_reset_input(oled, prompt);
                    sleep_ms(1000);  // Show reset message for 1 second
                }
            }
            // Invalid character (letters A, B, C, D)
            else if (key >= 'A' && key <= 'D') {
                // Show error for invalid characters
                screen_invalid_char_error(oled, prompt, input_buffer, display_message);
                sleep_ms(2000);  // Show error for 2 seconds
            }
        }
        
        sleep_ms(50);  // Small delay to avoid overwhelming the system
    }
}

int main()
{
    stdio_init_all();
    sleep_ms(1000); // Wait for the serial console to be ready
    //=======================================================
    // Screen initialization
    //=======================================================

    // I2C screen initialization
    i2c_setup(I2C_PORT, I2C_SDA, I2C_SCL); // Initialize I2C at 400kHz
    // Initialize the OLED display
    oled.external_vcc=false;
	bool res = ssd1306_init(&oled, 128, 64, 0x3c, i2c0);
    if(!res)printf("OLED initialization failed!\n");

    ssd1306_clear(&oled);
    ssd1306_draw_string(&oled, 0, 0, 1, "Hola!");
    ssd1306_show(&oled);
    //=======================================================
    // Keyboard initialization
    //=======================================================
    init_mat();
    //=======================================================

    uint8_t state = 0;
    uint8_t key=0;
    
    while (true) {
        char message[32];
        
        printf("Hello, world!\n");

        /**
         * TODO: Review how to wait for a key press
         */
        if(get_key_flag()){
            while (!read_mat(&key));
            printf("Key pressed: %c \n", key);
        }
        

        switch (state) {
            case 0: // Main menu
                screen_1(&oled);
                if (key == '1') {
                    state = 1; // Go to automatic mode
                    screen_2(&oled);
                } else if (key == '2') {
                    state = 2; // Go to manual mode
                    
                }
                break;
            case 1: // Automatic mode
                screen_2(&oled);
                if (key == '#') {
                    state = 0; // Go to main menu
                    screen_1(&oled);
                }
                break;
            case 2: // Manual mode
                // Adjust Ki
                ki = keyboard_to_float(&oled, "Adjust Ki");
                screen_4(&oled, "Ki", ki);
                
                /**
                 * TODO: Review how to wait for a key press
                 */

                // Wait for any key to continue
                key = 0;
                while (key == 0) {
                    if (get_key_flag()) {
                        while (!read_mat(&key));
                    }
                    sleep_ms(50);
                }
                
                // Adjust Kp
                kp = keyboard_to_float(&oled, "Adjust Kp");
                screen_4(&oled, "Kp", kp);
                
                // Wait for any key to continue
                key = 0;
                while (key == 0) {
                    if (get_key_flag()) {
                        while (!read_mat(&key));
                    }
                    sleep_ms(50);
                }
                
                // Adjust Kd
                kd = keyboard_to_float(&oled, "Adjust Kd");
                screen_4(&oled, "Kd", kd);
                
                // Wait for any key to continue
                key = 0;
                while (key == 0) {
                    if (get_key_flag()) {
                        while (!read_mat(&key));
                    }
                    sleep_ms(50);
                }
                
                // Adjust setpoint
                setpoint = keyboard_to_float(&oled, "Adjust setpoint");
                screen_4(&oled, "Setpoint", setpoint);
                
                // Wait for any key to continue
                key = 0;
                while (key == 0) {
                    if (get_key_flag()) {
                        while (!read_mat(&key));
                    }
                    sleep_ms(50);
                }
                
                // Show all parameters summary
                screen_params_summary(&oled, ki, kp, kd, setpoint);

                key = 0;
                while (key == 0) {
                    if (get_key_flag()) {
                        while (!read_mat(&key));
                        // Wait for # to return to main menu
                        if (key == '#') {
                            state = 0;
                            screen_1(&oled);
                        }
                    }
                    sleep_ms(50);
                }            
                            
                break;
            default:
                state = 0; // Reset to main menu if an unknown state is reached
                break;
        }
        

        sleep_ms(100);
    }
}
