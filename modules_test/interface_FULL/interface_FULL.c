/**
 * @file interface_FULL.c
 * @brief Main application file for the Delta TVC control system.
 * 
 * This file implements the main control loop for the Delta TVC system,
 * including state management, PID control, IMU data handling, and user interface
 * on an OLED display.
 * 
 * It integrates various modules such as MPU6050 for IMU data, PID control for servo angles,
 * ESC control for motor speed, and an OLED display for user interaction.
 * 
 * It uses a state machine to manage different modes of operation:
 * 1. Main Menu
 * 2. PID Control Mode
 * 3. Manual Parameter Adjustment Mode
 * 
 * @author Juan José Diaz, Juan Esteban Garcia, Santiago Vargas
 * 
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"

#include "lib/oled_screen/ssd1306.h"
#include "lib/oled_screen/screen.h"

#include "lib/keyboard/mat.h"
#include "lib/I2C_driver/i2c_driver.h"

#include "lib/mpu6050/mpu6050.h"
#include "lib/kalman_filter/kalman_filter.h"

#include "lib/servo_lib/servo_lib.h"
#include "lib/pid_lib/pid_lib.h"

#include "lib/ESC_lib/ESC_lib.h"

#include "lib/ADC/adc.h"

#include <string.h>
#include <stdlib.h>

// I2C IMU
#define I2C_PORT i2c1 ///< I2C port for MPU6050
#define I2C_SDA 14 ///< I2C SDA pin for MPU6050
#define I2C_SCL 15 ///< I2C SCL pin for MPU6050
// I2C Display
#define I2C_PORT_OLED i2c0 ///< I2C port for OLED display
#define I2C_SDA_OLED 20 ///< I2C SDA pin for OLED display
#define I2C_SCL_OLED 21 ///< I2C SCL pin for OLED display

#define MAX_INPUT_LENGTH 16  ///< Maximum characters for number input

#define INTERVALO_MS_UPDATE_SCREEN 1000 ///< Intervalo de tiempo para actualizar la pantalla

#define INTERVALO_MS_PID 100 ///< Intervalo para el PID

#define INTERVALO_MS_IMU 30 ///< Intervalo para leer el IMU

#define UPDATE_RATE_S (INTERVALO_MS_IMU / 1000.0f)  ///< Update rate in seconds

#define ROLL_SERVO_PIN 18 ///< GPIO pin for roll servo control
#define PITCH_SERVO_PIN 19 ///< GPIO pin for pitch servo control

#define PID_OFFSET 90 ///< Offset for servo angles to center them at 90 degrees
#define PITCH_OFFSET 110 ///< Offset for pitch servo angles to center them at 110 degrees

#define ROLL_SETPOINT 0 ///< Initial setpoint for roll angle in degrees
#define PITCH_SETPOINT 0 ///< Initial setpoint for pitch angle in degrees

#define ESC_PIN 10 ///< GPIO pin for ESC control

#define CURRENT_SENSOR_PIN 26 ///< GPIO pin for current sensor (ADC)

const float angle_roll_offset = -1.44f; ///< Offset for roll angle in degrees
const float angle_pitch_offset = 1.24f; ///< Offset for pitch angle in degrees

const float V_OFFSET = 1.65f;     ///< Asumimos Vcc = 3.3 V
const float SENSITIVITY = 0.185f;  ///< 185 mV/A para ACS712-05A

/// Timer para capturar datos desde el UART
static repeating_timer_t timer_update_screen;

///Timer para mandar señal servos
static repeating_timer_t timer_PID;

///Timer para leer IMU
static repeating_timer_t timer_imu;

adc_t adc_sensor; ///< ADC structure for current sensor

pid_controller_t pid_controller_roll; ///< PID controller for roll angle
pid_controller_t pid_controller_pitch; ///< PID controller for pitch angle
servo_t servo_roll; ///< Servo for roll control
servo_t servo_pitch; ///< Servo for pitch control
mpu6050_data_t mpu6050_data; ///< MPU6050 data structure

esc_t my_esc; ///< ESC structure for motor control
 
// Variables para las salidas del PID
float pid_output_roll = 0.0f; ///< PID output for roll angle
float pid_output_pitch = 0.0f; ///< PID output for pitch angle

ssd1306_t oled; ///< OLED display structure
uint8_t key = 0; ///< Variable to hold the key pressed
float ki = 0, kp = 0, kd = 0, setpoint = 0; ///< PID variables

uint8_t speed = 0; ///< Variable to hold ESC speed 

// Flags for state machine
volatile bool flag_read_imu = false; ///< Flag to indicate IMU data read needed
volatile bool flag_screen_value_update = false; ///< Flag to indicate screen update needed
volatile bool flag_pid_compute = false; ///< Flag to indicate PID computation needed

/** 
 * @brief Function to handle the main menu state.
 * This function displays the main menu on the OLED screen and waits for user input.
 * It allows the user to select different modes such as PID control, manual control.
*/
void StateMainMenu(void);

/**
 * @brief Function to handle the PID control state.
 * This function displays the accelerometer, angles, and current sensor values on the OLED screen.
 * It also updates the PID controller outputs and servo angles based on the IMU data.
 */
void StatePID(void);

/**
 * @brief This function allows the user to set the PID parameters and ESC speed through the keyboard.
 */
void StateManual(void);

/**
 * @brief Function to wait for a key press from the user.
 * This function displays a message on the OLED screen and waits for the user to press a key
 */
void wait_for_key_press(void);

/**
 * @brief Function to handle the PID compute callback.
 * This function is called periodically to compute the PID outputs for roll and pitch angles.
 */
bool timer_callback_PID(repeating_timer_t *rt);

/**
 * @brief Function to handle the IMU data reading callback.
 */
bool timer_callback_imu(repeating_timer_t *rt);

/**
 * @brief Function to handle the screen update callback.
 * This function is called periodically to update the OLED screen with the latest IMU data and current sensor values.
 */
bool timer_callback_screen(repeating_timer_t *rt);

/**
 * @brief Function to convert keyboard input to a float value.
 */
float keyboard_to_float(ssd1306_t *oled, const char *prompt);

void (*CurrentState)(void); ///< Pointer to the current state function

//========================================================

bool PID_callback(repeating_timer_t *rt) {
    // pid_compute(&pid_controller_roll);
    // pid_compute(&pid_controller_pitch);

    // servo_set_angle(&servo_roll, (uint8_t)(pid_output_roll + PID_OFFSET));
    // servo_set_angle(&servo_pitch, (uint8_t)(pid_output_pitch + PID_OFFSET));
    // //angulos del servo
    // printf("Roll Servo Angle: %d, Pitch Servo Angle: %d\n", 
    //     servo_roll.angle, servo_pitch.angle);

    flag_pid_compute = true;

    return true; // Return true to keep the timer running
}


bool timer_callback_imu(repeating_timer_t *rt) {
    // This function should handle IMU data reading logic

    flag_read_imu = true; // Set flag to indicate IMU data read needed
    return true; // Return true to keep the timer running
}

bool timer_callback_screen(repeating_timer_t *rt) {

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
    // Initialize the ESC
    //=======================================================
    esc_init(&my_esc, ESC_PIN, 50, 64.0f); // 50 Hz, clkdiv 64
    esc_write_speed(&my_esc, 0);
    sleep_ms(2000); // Wait for ESC to initialize

    //=======================================================
    // Initialize ADC for current sensor
    //=======================================================
    adc_util_init(&adc_sensor, CURRENT_SENSOR_PIN, 0, 3.3f);
    //=======================================================
    CurrentState = StateMainMenu;

    while (true) {
        CurrentState(); // Llamar estado actual
    }    
}


void StateMainMenu(void) {
    screen_1(&oled);

    if (get_key_flag()) {
        if (read_mat(&key)){
            printf("Key pressed: %c \n", key);

            if (key == '1') {
                screen_2(&oled);
                esc_write_speed(&my_esc, 32);
                add_repeating_timer_ms(INTERVALO_MS_UPDATE_SCREEN, timer_callback_screen, NULL, &timer_update_screen);
                add_repeating_timer_ms(INTERVALO_MS_PID, PID_callback, NULL, &timer_PID);
                add_repeating_timer_ms(INTERVALO_MS_IMU, timer_callback_imu, NULL, &timer_imu);
                CurrentState = StatePID;
            } else if (key == '2') {
                CurrentState = StateManual;
            }
        }
    }
}

void StatePID(void) {
    if(flag_screen_value_update) {
        flag_screen_value_update = false; // Reset the flag after updating the screen
        float current = adc_util_read_current_acs712(&adc_sensor, V_OFFSET, SENSITIVITY);
        //float current = 1.0f;
        screen_IMU_update(&oled, &mpu6050_data, angle_roll_offset, angle_pitch_offset, current);
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
        printf("IMU Data: Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", 
            mpu6050_data.AngleRoll, mpu6050_data.AnglePitch, mpu6050_data.AngleYaw);

    }
    if (flag_pid_compute){
        flag_pid_compute = false;
        pid_compute(&pid_controller_roll);
        pid_compute(&pid_controller_pitch);

        servo_set_angle(&servo_roll, (uint8_t)(pid_output_roll + PID_OFFSET));
        servo_set_angle(&servo_pitch, (uint8_t)(pid_output_pitch + PID_OFFSET));
        //angulos del servo
        // printf("Roll Servo Angle: %d, Pitch Servo Angle: %d\n", 
        //     servo_roll.angle, servo_pitch.angle);
        printf("IMU: Roll: %.2f, Pitch: %.2f\n", 
            mpu6050_data.KalmanAngleRoll, mpu6050_data.KalmanAnglePitch);
    }
    if (get_key_flag()) {
        if (read_mat(&key)){
            if (key == '#') {
                esc_write_speed(&my_esc, 0); // Stop the ESC
                cancel_repeating_timer(&timer_update_screen);
                cancel_repeating_timer(&timer_PID);
                cancel_repeating_timer(&timer_imu);
                screen_1(&oled);
                
                CurrentState = StateMainMenu;
            }
        }
    }
}

void wait_for_key_press() {
    key = 0;

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

    speed = (uint8_t)keyboard_to_float(&oled, "Adjust ESC speed");
    screen_4(&oled, "Speed", speed);
    wait_for_key_press();

    screen_params_summary(&oled, ki, kp, kd, setpoint, speed);

    while (!get_key_flag()){tight_loop_contents();}
    
    while (get_key_flag()) {
        while (!read_mat(&key));
        if (key == '*') {
            
            screen_1(&oled);
            CurrentState = StateMainMenu;
        } else if (key == '#') {
            pid_tune(&pid_controller_roll, kp, ki, kd);
            pid_tune(&pid_controller_pitch, kp, ki, kd);
            esc_write_speed(&my_esc, speed);
            screen_2(&oled);
            add_repeating_timer_ms(INTERVALO_MS_UPDATE_SCREEN, timer_callback_screen, NULL, &timer_update_screen);
            add_repeating_timer_ms(INTERVALO_MS_PID, PID_callback, NULL, &timer_PID);
            add_repeating_timer_ms(INTERVALO_MS_IMU, timer_callback_imu, NULL, &timer_imu);
            CurrentState = StatePID;
        } else {
            while (!get_key_flag()){tight_loop_contents();}

        }
    }
    
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
            if (read_mat(&key)){
            
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
                    sleep_ms(1000);  // Show reset message for 1 second
    
                    // Redraw initial screen
                    screen_initial_float_conversion(oled, prompt);
                }
            }
            // Invalid character (letters A, B, C)
            else if (key >= 'A' && key <= 'C') {
                // Show error for invalid characters
                screen_invalid_char_error(oled, prompt, input_buffer, display_message);

                sleep_ms(2000);  // Show error for 2 seconds

                // Redraw current input
                screen_update_float_conversion(oled, prompt, input_buffer, display_message);
            } else if (key == 'D') {
                // Clear input buffer
                memset(input_buffer, 0, sizeof(input_buffer));
                buffer_index = 0;
                has_decimal = false;
                
                // Show reset message
                //screen_reset_input(oled, prompt);
                //sleep_ms(1000);  // Show reset message for 1 second

                // Redraw initial screen
                screen_initial_float_conversion(oled, prompt);
            }
        }
        }else {
                tight_loop_contents(); // Wait for key press
            }
        }
}
