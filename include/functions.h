#ifndef AIRBURST_EXE_FUNCTIONS_H
#define AIRBURST_EXE_FUNCTIONS_H

#include <stdbool.h>
#include <time.h>

#include "imu.h"

// Stability check (evaluated by airburst_fall_state() at the control rate)
#define STABLE_ACCEL_MIN_G 0.8
#define STABLE_ACCEL_MAX_G 1.2
#define STABLE_MEASUREMENT_COUNT 10
#define RADAR_ANGLE_THRESHOLD_DEG 5.0
#define STABLE_GYRO_THRESHOLD_RAD 0.261799

// Accelerometer offsets (g), measured for this IMU and subtracted from every sample
#define ACCEL_OFFSET_X_G 0.023
#define ACCEL_OFFSET_Y_G 0.051
#define ACCEL_OFFSET_Z_G 0.0

// EKF accelerometer gate: the accelerometer is only a gravity reference when
// |a| is close to 1 g. Outside 1 g +/- ACCEL_GATE_G the EKF uses the gyro only.
#define ACCEL_GATE_G 0.1

// IMU mounting convention (used by the EKF measurement model).
// The value is the accelerometer Z reading, in g, when the radar is level (0 deg):
//   -1.0 -> IMU Z axis points down when level (accel_z reads about -1 g)  [previous behavior]
//   +1.0 -> IMU Z axis points up when level   (accel_z reads about +1 g)
// If the radar angle reads about 180 deg while level, flip this sign.
#define IMU_GRAVITY_SIGN (-1.0)

#define RAD_TO_DEG (180.0 / 3.14159265358979323846)

typedef enum {
    UNSTABLE,
    STABLE,
    MEASURING
} airburst_state_t;


airburst_state_t airburst_fall_state(double accel_magnitude_g, double gyro_magnitude_rad, double radar_angle_deg);

double calculate_dt(struct timespec *previous_time, const struct timespec *current_time);
bool calculate_gyro_bias(double *gyro_bias_x, double *gyro_bias_y, double *gyro_bias_z);
void quaternion_prediction_stage_1(double *q_w, double *q_x, double *q_y, double *q_z, double g_x, double g_y, double g_z, double dt);
void measurement_prediction_stage_2(double q_w, double q_x, double q_y, double q_z, double *predicted_g_x, double *predicted_g_y, double *predicted_g_z);
void acceleration_measurement_stage_3(double a_x, double a_y, double a_z, double *z_x, double *z_y, double *z_z);
void residual_stage_4(double z_x, double z_y, double z_z, double predicted_g_x, double predicted_g_y, double predicted_g_z, double *residual_x, double *residual_y, double *residual_z);
void measurement_jacobian_h_stage_5(double q_w, double q_x, double q_y, double q_z, double H[3][4]);
void state_transition_jacobian_f_stage_6(double g_x, double g_y, double g_z, double dt, double F[4][4]);
void covariance_prediction_stage_6(double P[4][4], double F[4][4], double Q[4][4]);
void measurement_noise_stage_7(double R[3][3]);
void innovation_covariance_stage_8(double H[3][4], double P[4][4], double R[3][3], double S[3][3]);
bool kalman_gain_stage_9(double P[4][4], double H[3][4], double S[3][3], double K[4][3]);
void quaternion_correction_stage_10(double *q_w, double *q_x, double *q_y, double *q_z, double K[4][3], double residual_x, double residual_y, double residual_z);
void covariance_update_stage_11(double P[4][4], double K[4][3], double H[3][4]);
bool ekf_update(double *q_w, double *q_x, double *q_y, double *q_z, double P[4][4], double Q[4][4], double R[3][3], double H[3][4], double F[4][4], double S[3][3], double K[4][3], double g_x, double g_y, double g_z, double a_x, double a_y, double a_z, double dt);
double calculate_radar_angle(double q_w, double q_x, double q_y, double q_z);
#endif // AIRBURST_EXE_FUNCTIONS_H
