#ifndef _MPU6050_KALMAN_H
#define _MPU6050_KALMAN_H

typedef struct {
	/* Private */
	float Q_angle; // Process noise variance for the accelerometer
	float Q_bias; // Process noise variance for the gyro bias
	float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

	float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
	float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
	float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

	float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
	float gain[2]; // Kalman gain - This is a 2x1 vector
	float y; // Angle difference
	float S; // Estimate error
} Kalman;

void initKalman(Kalman *K);
void setAngle(Kalman *K, float nAngle);
float getAngle(Kalman *K, float newAngle, float newRate, float dt);
float getRate(Kalman *K);

#endif
