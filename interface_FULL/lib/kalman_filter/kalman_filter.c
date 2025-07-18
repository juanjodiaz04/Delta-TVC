/**
 * @file kalman_filter.c
 * @brief Interfaz para el filtro de Kalman 1D.
 */

#include <stdint.h>
#include "kalman_filter.h"

void kalman_1d(float *KalmanState, float *KalmanUncertainty, float KalmanInput, float KalmanMeasurement, float dt) {

  *KalmanState = *KalmanState + dt * KalmanInput;
  *KalmanUncertainty = *KalmanUncertainty + dt * dt * 4 * 4; // supose std dev of rate = 4°/s 
  float KalmanGain = *KalmanUncertainty * 1/(1 * *KalmanUncertainty + 3 * 3); //supose std dev angle = 3°
  *KalmanState = *KalmanState + KalmanGain * (KalmanMeasurement - *KalmanState);
  *KalmanUncertainty = (1 - KalmanGain) * (*KalmanUncertainty);

}