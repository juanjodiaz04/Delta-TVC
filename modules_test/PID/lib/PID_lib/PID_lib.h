/**
 * @file PID_lib.h
 * @brief PID controller library for embedded systems using floating point math.
 *
 * This library implements a standard PID (Proportional-Integral-Derivative) controller.
 * Designed to be modular and portable for use in embedded platforms like the Raspberry Pi Pico.
 *
 * The controller uses pointer-based input/output for real-time integration with hardware sensors and actuators.
 * Timing is handled in milliseconds using the `pico/time.h` interface.
 *
 * Example of usage:
 * @code
 * float input, output, setpoint;
 * struct pid_controller my_pid;
 * pid_create(&my_pid, &input, &output, &setpoint, 1.0f, 0.1f, 0.05f);
 * pid_auto(&my_pid);
 * @endcode
 */

#ifndef PID_LIB_H
#define PID_LIB_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @enum pid_control_direction_t
 * @brief Control direction of the PID output.
 *
 * - E_PID_DIRECT: A positive error increases the output.
 * - E_PID_REVERSE: A positive error decreases the output.
 */
typedef enum {
    E_PID_DIRECT,
    E_PID_REVERSE,
} pid_control_direction_t;

/**
 * @struct pid_controller_t
 * @brief Structure that holds all state and parameters for the PID controller.
 */
typedef struct {
    float *input;     /**< Pointer to measured input value (e.g., from a sensor) */
    float *output;    /**< Pointer to controller output value */
    float *setpoint;  /**< Pointer to target setpoint value */

    float Kp;         /**< Proportional gain */
    float Ki;         /**< Integral gain (scaled by sample time) */
    float Kd;         /**< Derivative gain (scaled by sample time) */

    float omin;       /**< Minimum allowed output */
    float omax;       /**< Maximum allowed output */

    float iterm;      /**< Integral accumulator */
    float lastin;     /**< Previous input, used for derivative */

    uint16_t sampletime;   /**< Sample time in milliseconds */

    bool automode;          /**< PID mode: true = automatic, false = manual */
    pid_control_direction_t direction; /**< Control direction */
} pid_controller_t;


/**
 * @brief Creates and initializes a new PID controller instance.
 * 
 * @param pid Pointer to a pid_controller_t structure.
 * @param in Pointer to the input variable.
 * @param out Pointer to the output variable.
 * @param set Pointer to the setpoint variable.
 * @param kp Proportional gain.
 * @param ki Integral gain.
 * @param kd Derivative gain.
 */
void pid_create(pid_controller_t *pid, float *in, float *out, float *set, float kp, float ki, float kd);

/**
 * @brief Computes a new PID output if the sample time has elapsed.
 *
 * This function should be called periodically to update the output value based on the input and setpoint.
 * It internally checks whether the sample time has elapsed.
 *
 * @param pid Pointer to the PID controller.
 */
void pid_compute(pid_controller_t *pid);

/**
 * @brief Updates the PID tuning parameters.
 * 
 * @param pid Pointer to the PID controller.
 * @param kp New proportional gain.
 * @param ki New integral gain.
 * @param kd New derivative gain.
 */
void pid_tune(pid_controller_t *pid, float kp, float ki, float kd);

/**
 * @brief Sets a new sample time for the PID loop.
 * 
 * @param pid Pointer to the PID controller.
 * @param time_ms Sample time in milliseconds.
 */
void pid_sample(pid_controller_t *pid, uint32_t time_ms);

/**
 * @brief Sets the output limits of the PID controller.
 * 
 * @param pid Pointer to the PID controller.
 * @param min Minimum output value.
 * @param max Maximum output value.
 */
void pid_limits(pid_controller_t *pid, float min, float max);

/**
 * @brief Enables automatic mode for the PID controller.
 * 
 * When enabled, `pid_compute()` updates the output based on PID logic.
 * This should be called after manually adjusting output or at system start.
 *
 * @param pid Pointer to the PID controller.
 */
void pid_auto(pid_controller_t *pid);

/**
 * @brief Disables automatic mode and enters manual control mode.
 * 
 * In manual mode, `pid_compute()` will have no effect.
 *
 * @param pid Pointer to the PID controller.
 */
void pid_manual(pid_controller_t *pid);

/**
 * @brief Sets the control direction of the PID controller.
 * 
 * @param pid Pointer to the PID controller.
 * @param dir E_PID_DIRECT or E_PID_REVERSE.
 */
void pid_direction(pid_controller_t *pid, pid_control_direction_t dir);

#endif // PID_LIB_H
