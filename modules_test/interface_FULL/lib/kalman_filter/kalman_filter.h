/**
 * @file kalman_filter.h
 * @brief Interfaz para el filtro de Kalman 1D.
 */
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <stdint.h>

/* This code is part of the Kalman filter provided by 
https://github.com/CarbonAeronautics/Part-XV-1DKalmanFilter/blob/main/ArduinoCode
*/
/**
 * @brief 1D Kalman filter function.
 * This function implements a simple 1D Kalman filter to estimate the state of a system.
 * @param KalmanState Pointer to the current state estimate (angle).
 * @param KalmanUncertainty Pointer to the current uncertainty estimate.
 * @param KalmanInput The input variable (rate).
 * @param KalmanMeasurement The measurement variable (angle).
 * @param dt The time step for the filter update.
 * @return void
 * 
 */
void kalman_1d(float *KalmanState, float *KalmanUncertainty, float KalmanInput, float KalmanMeasurement, float dt);

#endif