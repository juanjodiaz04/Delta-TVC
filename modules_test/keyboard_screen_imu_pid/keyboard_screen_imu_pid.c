#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "lib/ssd1306.h"
#include "lib/mat.h"
#include "lib/I2C_driver/i2c_driver.h"

#include "lib/mpu6050/mpu6050.h"
#include "lib/kalman_filter/kalman_filter.h"

#include "lib/servo_lib/servo_lib.h"
#include "lib/pid_lib/pid_lib.h"

#include <string.h>  // Added for string functions
#include <stdlib.h>

// I2C IMU
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
// I2C Display
#define I2C_PORT_OLED i2c0
#define I2C_SDA_OLED 20
#define I2C_SCL_OLED 21

#define MAX_INPUT_LENGTH 16  // Maximum characters for number input

/// Intervalo de tiempo para actualizar la pantalla
#define INTERVALO_MS_UPDATE_SCREEN 1000

/// Intervalo para actualizar los servos
#define INTERVALO_MS_PID 100

///Intervalo para leer IMU
#define INTERVALO_MS_IMU 10

#define UPDATE_RATE_S (INTERVALO_MS_IMU / 1000.0f)  // Update rate in seconds

#define ROLL_SERVO_PIN 18
#define PITCH_SERVO_PIN 19

#define PID_OFFSET 90
#define PITCH_OFFSET 110

#define ROLL_SETPOINT 0
#define PITCH_SETPOINT 0

const float angle_roll_offset = 3.4f;
const float angle_pitch_offset = -0.3f;

/// Valor de prueba para la pantalla
volatile float valor_prueba = 0.0f;

/// Timer para capturar datos desde el UART
static repeating_timer_t timer_update_screen;

///Timer para mandar señal servos
static repeating_timer_t timer_PID;

///Timer para leer IMU
static repeating_timer_t timer_imu;

ssd1306_t oled;

pid_controller_t pid_controller_roll;
pid_controller_t pid_controller_pitch;
servo_t servo_roll;
servo_t servo_pitch;
mpu6050_data_t mpu6050_data;

// Variables para las salidas del PID
float pid_output_roll = 0.0f;
float pid_output_pitch = 0.0f;

float ki, kp, kd, setpoint; // PID variables

// Global variables for the state machine
ssd1306_t oled;
uint8_t key = 0;
float ki = 0, kp = 0, kd = 0, setpoint = 0;

// Flags for state machine
volatile bool flag_read_imu = false;
volatile bool flag_screen_value_update = false;

// Function prototypes
void StateMainMenu(void);
void StatePID(void);
void StateManual(void);
void wait_for_key_press(void);

// Screen function prototypes
void screen_1(ssd1306_t *oled);
void screen_2(ssd1306_t *oled);
void screen_IMU_update(ssd1306_t *oled);
void screen_3(ssd1306_t *oled, const char *message);
void screen_4(ssd1306_t *oled, const char *param_name, float value);
void screen_initial_float_conversion(ssd1306_t *oled, const char *prompt);
void screen_update_float_conversion(ssd1306_t *oled, const char *prompt, const char *input_buffer, char *display_message);
void screen_confirmation(ssd1306_t *oled, const char *message, float result, char *display_message);
void screen_params_summary(ssd1306_t *oled, float ki, float kp, float kd, float setpoint);
void screen_buffer_error(ssd1306_t *oled);
void screen_decimal_exists_error(ssd1306_t *oled);
void screen_decimal_before_number_error(ssd1306_t *oled);
void screen_number_too_long_error(ssd1306_t *oled);
void screen_reset_input(ssd1306_t *oled, const char *prompt);
void screen_invalid_char_error(ssd1306_t *oled, const char *prompt, const char *input_buffer, char *display_message);

// Timer callback prototypes
bool timer_callback_PID(repeating_timer_t *rt);
bool timer_callback_imu(repeating_timer_t *rt);
bool timer_callback_screen(repeating_timer_t *rt);

// Utility function prototypes
float keyboard_to_float(ssd1306_t *oled, const char *prompt);

// Puntero a la función de estado actual
void (*CurrentState)(void);

//========================================================

bool PID_callback(repeating_timer_t *rt) {
    pid_compute(&pid_controller_roll);
    pid_compute(&pid_controller_pitch);
    // servo_set_angle(&servo_roll, (uint8_t)(*pid_controller_roll.output + ROLL_OFFSET));
    // servo_set_angle(&servo_pitch, (uint8_t)(*pid_controller_pitch.output + PITCH_OFFSET));
    servo_set_angle(&servo_roll, (uint8_t)(pid_output_roll + PID_OFFSET));
    servo_set_angle(&servo_pitch, (uint8_t)(pid_output_pitch + PID_OFFSET));
    //printf(" Roll:%f, Pitch:%f\n", *(pid_controller_roll.output) + ROLL_OFFSET, *(pid_controller_pitch.output) + PITCH_OFFSET);
    //printf(" Roll:%f, Pitch:%f\n", pid_output_roll + PID_OFFSET, pid_output_pitch + PID_OFFSET);
    //angulos del servo
    printf("Roll Servo Angle: %d, Pitch Servo Angle: %d\n", 
        servo_roll.angle, servo_pitch.angle);

    return true; // Return true to keep the timer running
}


bool timer_callback_imu(repeating_timer_t *rt) {
    // This function should handle IMU data reading logic

    flag_read_imu = true; // Set flag to indicate IMU data read needed
    return true; // Return true to keep the timer running
}


/**
 * TODO: Cuando tengamos implementado la comunicación entre PICOs
 * esta función debe ser llamada cada cierto tiempo, se supone que ya
 * se capturaron los datos de la IMU y se guardaron en alguna variable
 * global, por lo que aquí solo se actualiza la pantalla.
 */


bool timer_callback_screen(repeating_timer_t *rt) {
    printf("Updating screen with valor_prueba = %.2f\n", valor_prueba); // Debug

    flag_screen_value_update = true; // Set flag to indicate screen update needed
    return true; // Return true to keep the timer running
}

int main()
{
    stdio_init_all();
    sleep_ms(1000); // Wait for the serial console to be ready
    //=======================================================
    // Initialize I2C for MPU6050
    mpu6050_init(I2C_PORT, I2C_SDA, I2C_SCL, &mpu6050_data); // Initialize MPU6050
    servo_init(&servo_roll, ROLL_SERVO_PIN, 90);
    servo_init(&servo_pitch, PITCH_SERVO_PIN, 90);

    servo_start(&servo_roll, 90); // Start the servo at 90 degrees
    servo_start(&servo_pitch, 90); // Start the servo at 90 degrees

    // pid_create(&pid_controller_roll, &mpu6050_data.KalmanAngleRoll, &servo_roll.angle, ROLL_SETPOINT, 100.0f, 0.0f, 0.0f, -20.0f, 20.0f);
    // pid_create(&pid_controller_pitch, &mpu6050_data.KalmanAnglePitch, &servo_pitch.angle, PITCH_SETPOINT, 100.0f, 0.0f, 0.0f, -30.0f, 30.0f);
    pid_create(&pid_controller_roll, &mpu6050_data.KalmanAngleRoll, &pid_output_roll, ROLL_SETPOINT, 
        30.0f, 3.0f, 0.0f, -30.0f, 10.0f);
    pid_create(&pid_controller_pitch, &mpu6050_data.KalmanAnglePitch, &pid_output_pitch, PITCH_SETPOINT, 
        30.0f, 3.0f, 0.0f, -10.0f, 50.0f);

    pid_set_angle_offset(&pid_controller_roll, angle_roll_offset);
    pid_set_angle_offset(&pid_controller_pitch, angle_pitch_offset);

    printf("Hello, MPU6050! Reading raw data from registers...\n");

    float RateCalibrationRoll = 0.0f, RateCalibrationPitch = 0.0f, RateCalibrationYaw = 0.0f;

    mpu6050_calibrate_gyro(&mpu6050_data, 1000); // Calibrate gyroscope offsets

    printf("Calibration complete. Average rates:\n");
    printf("Roll: %f, Pitch: %f, Yaw: %f\n", 
            mpu6050_data.gyro_offset_x, mpu6050_data.gyro_offset_y, mpu6050_data.gyro_offset_z);

    //=======================================================
    // Screen initialization
    //=======================================================

    // I2C screen initialization
    i2c_setup(I2C_PORT_OLED, I2C_SDA_OLED, I2C_SCL_OLED); // Initialize I2C at 400kHz
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

    CurrentState = StateMainMenu;

    while (true) {
        CurrentState(); // Llamar estado actual
    }    
}


void StateMainMenu(void) {
    screen_1(&oled);

    if (get_key_flag()) {
        while (!read_mat(&key));
        printf("Key pressed: %c \n", key);

        if (key == '1') {
            screen_2(&oled);
            add_repeating_timer_ms(INTERVALO_MS_UPDATE_SCREEN, timer_callback_screen, NULL, &timer_update_screen);
            add_repeating_timer_ms(INTERVALO_MS_PID, PID_callback, NULL, &timer_PID);
            add_repeating_timer_ms(INTERVALO_MS_IMU, timer_callback_imu, NULL, &timer_imu);
            CurrentState = StatePID;
        } else if (key == '2') {
            CurrentState = StateManual;
        }
    }
}

void StatePID(void) {
    if(flag_screen_value_update) {
        flag_screen_value_update = false; // Reset the flag after updating the screen
        screen_IMU_update(&oled);
    }
    if(flag_read_imu) {
        flag_read_imu = false; // Reset the flag after reading IMU
        mpu6050_read(&mpu6050_data); // Read data from MPU6050

        kalman_1d(&(mpu6050_data.KalmanAngleRoll), &(mpu6050_data.KalmanUncertaintyAngleRoll), 
            mpu6050_data.RateRoll, mpu6050_data.AngleRoll,
            UPDATE_RATE_S);
        kalman_1d(&(mpu6050_data.KalmanAnglePitch), &(mpu6050_data.KalmanUncertaintyAnglePitch),
            mpu6050_data.RatePitch, mpu6050_data.AnglePitch,
            UPDATE_RATE_S);
        valor_prueba += 0.1f; // Simulate some data capture
        if(valor_prueba > 200.0f) {
            valor_prueba = 0.0f; // Reset after reaching a threshold
        }
    }
    if (get_key_flag()) {
        while (!read_mat(&key));
        if (key == '#') {
            cancel_repeating_timer(&timer_update_screen);
            cancel_repeating_timer(&timer_PID);
            cancel_repeating_timer(&timer_imu);
            screen_1(&oled);
            
            CurrentState = StateMainMenu;
        }
    }
}

void wait_for_key_press() {
    key = 0;
    // while (key == 0) {
    //     if (get_key_flag()) {
    //         while (!read_mat(&key));
    //     }
    //     sleep_ms(50);
    // }
    while (!get_key_flag()){tight_loop_contents();}
    if(get_key_flag()) {
        while (!read_mat(&key));
    }
}

void StateManual(void) {
    ki = keyboard_to_float(&oled, "Adjust Ki");
    screen_4(&oled, "Ki", ki);
    wait_for_key_press();

    kp = keyboard_to_float(&oled, "Adjust Kp");
    screen_4(&oled, "Kp", kp);
    wait_for_key_press();

    kd = keyboard_to_float(&oled, "Adjust Kd");
    screen_4(&oled, "Kd", kd);
    wait_for_key_press();

    setpoint = keyboard_to_float(&oled, "Adjust setpoint");
    screen_4(&oled, "Setpoint", setpoint);
    wait_for_key_press();

    screen_params_summary(&oled, ki, kp, kd, setpoint);

    while (!get_key_flag()){tight_loop_contents();}
    if (get_key_flag()) {
        while (!read_mat(&key));
        if (key == '*') {
            screen_1(&oled);
            CurrentState = StateMainMenu;
        } else if (key == '#') {
            pid_tune(&pid_controller_roll, kp, ki, kd);
            pid_tune(&pid_controller_pitch, kp, ki, kd);
            screen_2(&oled);
            add_repeating_timer_ms(INTERVALO_MS_UPDATE_SCREEN, timer_callback_screen, NULL, &timer_update_screen);
            add_repeating_timer_ms(INTERVALO_MS_PID, PID_callback, NULL, &timer_PID);
            add_repeating_timer_ms(INTERVALO_MS_IMU, timer_callback_imu, NULL, &timer_imu);
            CurrentState = StatePID;
        }
    }
    
}

































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

void screen_IMU_update(ssd1306_t *oled){
    // ssd1306_clear(oled);
    // ssd1306_draw_string(oled, 0, 0, 1, "Automatic Mode");
    // char buffer[32];
    // snprintf(buffer, sizeof(buffer), "Ac_x = %.2f", valor_prueba);
    // ssd1306_draw_string(oled, 0, 10, 1, buffer);
    // snprintf(buffer, sizeof(buffer), "Ac_y = %.2f", valor_prueba*2);
    // ssd1306_draw_string(oled, 0, 20, 1, buffer);
    // snprintf(buffer, sizeof(buffer), "Gy_x = %.2f", valor_prueba*3);
    // ssd1306_draw_string(oled, 0, 40, 1, buffer);
    // snprintf(buffer, sizeof(buffer), "Gy_y = %.2f", valor_prueba*4);
    // ssd1306_draw_string(oled, 0, 50, 1, buffer);
    // ssd1306_show(oled);

    char buffer[32];
    ssd1306_clear_square(oled, 35, 10, 64, 54); // Clear the area for new data
    snprintf(buffer, sizeof(buffer), " %.2f", mpu6050_data.AccX);
    ssd1306_draw_string(oled, 35, 20, 1, buffer);
    snprintf(buffer, sizeof(buffer), " %.2f", mpu6050_data.AccY);
    ssd1306_draw_string(oled, 35, 30, 1, buffer);

    //Angulos
    snprintf(buffer, sizeof(buffer), " %.2f", mpu6050_data.KalmanAngleRoll + angle_roll_offset);
    ssd1306_draw_string(oled, 35, 45, 1, buffer);
    snprintf(buffer, sizeof(buffer), " %.2f", mpu6050_data.KalmanAnglePitch + angle_pitch_offset);
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
                    input_buffer[buffer_index] = '\0';  
                    
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
                    
                    sleep_ms(200);  // Show error
                    
                    // Redraw current input
                    screen_update_float_conversion(oled, prompt, input_buffer, display_message);
                }
            }
            // Check if it's confirm (#)
            else if (key == '#') {
                if (buffer_index > 0) {
                    // Check if the last character is a decimal point
                    if (input_buffer[buffer_index - 1] == '.') {
                        // Remove decimal point
                        input_buffer[buffer_index - 1] = '\0';
                        buffer_index--;
                        has_decimal = false;
                    }
                    
                    // Convert to float
                    float result = atof(input_buffer);
                    
                    // Show confirmation
                    screen_confirmation(oled, prompt, result, display_message);
                    sleep_ms(500);  // Show confirmation
                    
                    return result;
                } else {
                    // Empty input, reset
                    memset(input_buffer, 0, sizeof(input_buffer));
                    buffer_index = 0;
                    has_decimal = false;
                    
                    // Show reset message
                    screen_reset_input(oled, prompt);
                    sleep_ms(150);  // Show reset message for 1 second
                }
            }
            // Invalid character (letters A, B, C, D)
            else if (key >= 'A' && key <= 'D') {
                // Show error for invalid characters
                screen_invalid_char_error(oled, prompt, input_buffer, display_message);
                sleep_ms(200);  // Show error for 2 seconds
            }
        }
        
        sleep_ms(50);  
    }
}
