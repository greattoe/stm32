/*
 * kalman_filter.h
 *
 * Created on: 2025. 04. 15
 *
 * KALMAN Filter library for Visualizing MPU6050 Data
 */

#ifndef __KALMAN_FILTER_H_
#define __KALMAN_FILTER_H_

typedef struct {
    float Q_angle;      // Process noise variance for the accelerometer
    float Q_bias;       // Process noise variance for the gyro bias
    float R_measure;    // Measurement noise variance

    float angle;        // The angle calculated by the Kalman filter
    float bias;         // The gyro bias calculated by the Kalman filter
    float rate;         // Unbiased rate calculated from the rate and the calculated bias

    float P[2][2];      // Error covariance matrix
} Kalman_t;

void Kalman_Init(Kalman_t *kalman);
float Kalman_GetAngle(Kalman_t *kalman, float newAngle, float newRate, float dt);

#endif // __KALMAN_FILTER_H_
