#ifndef AIRBURST_EXE_FUNCTIONS_H
#define AIRBURST_EXE_FUNCTIONS_H

#include <time.h>
#include "../subprojects/imu_lib/include/imu.h"

#define STABLE_ACCEL_MIN_G 0.8
#define STABLE_ACCEL_MAX_G 1.2
#define ACCEL_THRESHOLD_VALUE_G 0.2
#define STABLE_MEASUREMENT_COUNT 10
#define RADAR_ANGLE_THRESHOLD_DEG 5.0
#define STABLE_GYRO_THRESHOLD_RAD 0.261799

#define M_PI 3.14159265358979323846
typedef enum {
    UNSTABLE,
    STABLE,
    MEASURING,
    BURST
} airburst_state_t;


airburst_state_t airburst_fall_state(accel_value_t accel_value, gyro_value_t gyro_value, double radar_angle);

double calculate_dt(struct timespec *previous_time, const struct timespec *current_time);
void calculate_gyro_bias(double *gyro_bias_x, double *gyro_bias_y, double *gyro_bias_z);
void quaternion_prediction_stage_1(double *q_w, double *q_x, double *q_y, double *q_z, double g_x, double g_y, double g_z, double dt);
void measurement_prediction_stage_2(double q_w, double q_x, double q_y, double q_z, double *predicted_g_x, double *predicted_g_y, double *predicted_g_z);
void acceleration_measurement_stage_3(double a_x, double a_y, double a_z, double *z_x, double *z_y, double *z_z);
void residual_stage_4(double z_x, double z_y, double z_z, double predicted_g_x, double predicted_g_y, double predicted_g_z, double *residual_x, double *residual_y, double *residual_z);
void measurement_jacobian_h_stage_5(double q_w, double q_x, double q_y, double q_z, double H[3][4]);
void state_transition_jacobian_f_stage_6(double g_x, double g_y, double g_z, double dt, double F[4][4]);
void covariance_prediction_stage_6(double P[4][4], double F[4][4], double Q[4][4]);
void measurement_noise_stage_7(double R[3][3]);
void innovation_covariance_stage_8(double H[3][4], double P[4][4], double R[3][3], double S[3][3]);
void kalman_gain_stage_9(double P[4][4], double H[3][4], double S[3][3], double K[4][3]);
void quaternion_correction_stage_10(double *q_w, double *q_x, double *q_y, double *q_z, double K[4][3], double residual_x, double residual_y, double residual_z);
void covariance_update_stage_11(double P[4][4], double K[4][3], double H[3][4]);
void ekf_update(double *q_w, double *q_x, double *q_y, double *q_z, double P[4][4], double Q[4][4], double R[3][3], double H[3][4], double F[4][4], double S[3][3], double K[4][3], double g_x, double g_y, double g_z, double a_x, double a_y, double a_z, double dt);
double calculate_radar_angle(double q_w, double q_x, double q_y, double q_z);
#endif // AIRBURST_EXE_FUNCTIONS_H