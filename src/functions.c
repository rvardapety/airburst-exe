#include "../include/functions.h"

#include <stdint.h>
#include <math.h>
#include <unistd.h>

uint8_t count = 0;
airburst_state_t current_state = UNSTABLE;

// AIRBURST FALL / STABILITY STATE
airburst_state_t airburst_fall_state(accel_value_t accel_value, gyro_value_t gyro_value, double radar_angle)
{
    double gx = (double)gyro_value.gyro_x;
    double gy = (double)gyro_value.gyro_y;
    double gz = (double)gyro_value.gyro_z;

    double ax = (double)accel_value.accel_x;
    double ay = (double)accel_value.accel_y;
    double az = (double)accel_value.accel_z;

    double gyro_magnitude = sqrt(gx * gx + gy * gy + gz * gz);
    double accel_magnitude = sqrt(ax * ax + ay * ay + az * az);

    if (accel_magnitude < ACCEL_THRESHOLD_VALUE_G) {
        count = 0;
        current_state = UNSTABLE;
        return current_state;
    }

    if (accel_magnitude >= STABLE_ACCEL_MIN_G &&
        accel_magnitude < STABLE_ACCEL_MAX_G &&
        gyro_magnitude < STABLE_GYRO_THRESHOLD_RAD &&
        radar_angle <= RADAR_ANGLE_THRESHOLD_DEG) {
        count++;

        if (count >= STABLE_MEASUREMENT_COUNT) {
            if (current_state == UNSTABLE) {
                current_state = STABLE;
            } else if (current_state == STABLE) {
                current_state = MEASURING;
            }

            return current_state;
        }

        current_state = UNSTABLE;
        return current_state;
    }

    count = 0;
    current_state = UNSTABLE;
    return current_state;
}

// GYROSCOPE BIAS CALCULATION
void calculate_gyro_bias(double *gyro_bias_x, double *gyro_bias_y, double *gyro_bias_z)
{
    gyro_value_t gyro;

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;

    int measurement_count = 100;

    for (int i = 0; i < measurement_count; i++) {
        imu_get_gyro_data(&gyro);

        sum_x += (double)gyro.gyro_x;
        sum_y += (double)gyro.gyro_y;
        sum_z += (double)gyro.gyro_z;

        usleep(10000);
    }

    *gyro_bias_x = sum_x / measurement_count;
    *gyro_bias_y = sum_y / measurement_count;
    *gyro_bias_z = sum_z / measurement_count;
}

// DT CALCULATION
double calculate_dt(struct timespec *previous_time, const struct timespec *current_time)
{
    double dt = (double)(current_time->tv_sec - previous_time->tv_sec) + (double)(current_time->tv_nsec - previous_time->tv_nsec) / 1e9;

    *previous_time = *current_time;

    return dt;
}


// EKF STAGE 1
// Quaternion prediction
void quaternion_prediction_stage_1(double *q_w, double *q_x, double *q_y, double *q_z, double g_x, double g_y, double g_z, double dt)
{
    double alpha_w = -*q_x * g_x - *q_y * g_y - *q_z * g_z;
    double alpha_x = *q_w * g_x + *q_y * g_z - *q_z * g_y;
    double alpha_y = *q_w * g_y - *q_x * g_z + *q_z * g_x;
    double alpha_z = *q_w * g_z + *q_x * g_y - *q_y * g_x;

    double qw_pred = *q_w + 0.5 * alpha_w * dt;
    double qx_pred = *q_x + 0.5 * alpha_x * dt;
    double qy_pred = *q_y + 0.5 * alpha_y * dt;
    double qz_pred = *q_z + 0.5 * alpha_z * dt;

    double q_magnitude = sqrt(qw_pred * qw_pred + qx_pred * qx_pred + qy_pred * qy_pred + qz_pred * qz_pred);

    if (q_magnitude < 1e-12) {
        return;
    }

    *q_w = qw_pred / q_magnitude;
    *q_x = qx_pred / q_magnitude;
    *q_y = qy_pred / q_magnitude;
    *q_z = qz_pred / q_magnitude;
}

// EKF STAGE 2
// Predicted gravity measurement
void measurement_prediction_stage_2(double q_w, double q_x, double q_y, double q_z, double *predicted_g_x, double *predicted_g_y, double *predicted_g_z)
{
    *predicted_g_x = 2.0 * (q_x * q_z + q_w * q_y);
    *predicted_g_y = 2.0 * (q_y * q_z - q_w * q_x);
    *predicted_g_z = -(q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z);
}

// EKF STAGE 3
// Accelerometer measurement normalization
void acceleration_measurement_stage_3(double a_x, double a_y, double a_z, double *z_x, double *z_y, double *z_z)
{
    double accel_magnitude = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);

    if (accel_magnitude < 1e-9) {
        *z_x = 0.0;
        *z_y = 0.0;
        *z_z = 0.0;
        return;
    }

    *z_x = a_x / accel_magnitude;
    *z_y = a_y / accel_magnitude;
    *z_z = a_z / accel_magnitude;
}

// EKF STAGE 4
// Residual
void residual_stage_4(double z_x, double z_y, double z_z, double predicted_g_x, double predicted_g_y, double predicted_g_z, double *residual_x, double *residual_y, double *residual_z)
{
    *residual_x = z_x - predicted_g_x;
    *residual_y = z_y - predicted_g_y;
    *residual_z = z_z - predicted_g_z;
}

// EKF STAGE 5
// Measurement Jacobian H
void measurement_jacobian_h_stage_5(double q_w, double q_x, double q_y, double q_z, double H[3][4])
{
    H[0][0] = 2.0 * q_y;
    H[0][1] = 2.0 * q_z;
    H[0][2] = 2.0 * q_w;
    H[0][3] = 2.0 * q_x;

    H[1][0] = -2.0 * q_x;
    H[1][1] = -2.0 * q_w;
    H[1][2] = 2.0 * q_z;
    H[1][3] = 2.0 * q_y;

    H[2][0] = -2.0 * q_w;
    H[2][1] = 2.0 * q_x;
    H[2][2] = 2.0 * q_y;
    H[2][3] = -2.0 * q_z;
}

// EKF STAGE 6
// State transition Jacobian F
void state_transition_jacobian_f_stage_6(double g_x, double g_y, double g_z, double dt, double F[4][4])
{
    F[0][0] = 1.0;
    F[0][1] = -0.5 * g_x * dt;
    F[0][2] = -0.5 * g_y * dt;
    F[0][3] = -0.5 * g_z * dt;

    F[1][0] = 0.5 * g_x * dt;
    F[1][1] = 1.0;
    F[1][2] = 0.5 * g_z * dt;
    F[1][3] = -0.5 * g_y * dt;

    F[2][0] = 0.5 * g_y * dt;
    F[2][1] = -0.5 * g_z * dt;
    F[2][2] = 1.0;
    F[2][3] = 0.5 * g_x * dt;

    F[3][0] = 0.5 * g_z * dt;
    F[3][1] = 0.5 * g_y * dt;
    F[3][2] = -0.5 * g_x * dt;
    F[3][3] = 1.0;
}

// EKF STAGE 6
// Covariance prediction
void covariance_prediction_stage_6(double P[4][4], double F[4][4], double Q[4][4])
{
    double FP[4][4] = {{0.0}};
    double F_transpose[4][4] = {{0.0}};
    double P_predicted[4][4] = {{0.0}};

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            F_transpose[i][j] = F[j][i];
        }
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                FP[i][j] += F[i][k] * P[k][j];
            }
        }
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                P_predicted[i][j] += FP[i][k] * F_transpose[k][j];
            }
        }
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P[i][j] = P_predicted[i][j] + Q[i][j];
        }
    }
}

// EKF STAGE 7
// Measurement noise covariance R
void measurement_noise_stage_7(double R[3][3])
{
    R[0][0] = 0.01;
    R[0][1] = 0.0;
    R[0][2] = 0.0;

    R[1][0] = 0.0;
    R[1][1] = 0.01;
    R[1][2] = 0.0;

    R[2][0] = 0.0;
    R[2][1] = 0.0;
    R[2][2] = 0.01;
}

// EKF STAGE 8
// Innovation covariance S
void innovation_covariance_stage_8(double H[3][4], double P[4][4], double R[3][3], double S[3][3])
{
    double H_P[3][4] = {{0.0}};
    double H_transpose[4][3] = {{0.0}};

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            H_transpose[j][i] = H[i][j];
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                H_P[i][j] += H[i][k] * P[k][j];
            }
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            S[i][j] = R[i][j];

            for (int k = 0; k < 4; k++) {
                S[i][j] += H_P[i][k] * H_transpose[k][j];
            }
        }
    }
}

// EKF STAGE 9
// Kalman gain K
void kalman_gain_stage_9(double P[4][4], double H[3][4], double S[3][3], double K[4][3])
{
    double H_transpose[4][3] = {{0.0}};
    double P_H_transpose[4][3] = {{0.0}};
    double S_inverse[3][3] = {{0.0}};

    double determinant =
          S[0][0] * (S[1][1] * S[2][2] - S[1][2] * S[2][1])
        - S[0][1] * (S[1][0] * S[2][2] - S[1][2] * S[2][0])
        + S[0][2] * (S[1][0] * S[2][1] - S[1][1] * S[2][0]);

    if (fabs(determinant) < 1e-12) {
        return;
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            H_transpose[j][i] = H[i][j];
        }
    }

    S_inverse[0][0] = (S[1][1] * S[2][2] - S[1][2] * S[2][1]) / determinant;
    S_inverse[0][1] = (S[0][2] * S[2][1] - S[0][1] * S[2][2]) / determinant;
    S_inverse[0][2] = (S[0][1] * S[1][2] - S[0][2] * S[1][1]) / determinant;

    S_inverse[1][0] = (S[1][2] * S[2][0] - S[1][0] * S[2][2]) / determinant;
    S_inverse[1][1] = (S[0][0] * S[2][2] - S[0][2] * S[2][0]) / determinant;
    S_inverse[1][2] = (S[0][2] * S[1][0] - S[0][0] * S[1][2]) / determinant;

    S_inverse[2][0] = (S[1][0] * S[2][1] - S[1][1] * S[2][0]) / determinant;
    S_inverse[2][1] = (S[0][1] * S[2][0] - S[0][0] * S[2][1]) / determinant;
    S_inverse[2][2] = (S[0][0] * S[1][1] - S[0][1] * S[1][0]) / determinant;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 4; k++) {
                P_H_transpose[i][j] += P[i][k] * H_transpose[k][j];
            }
        }
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            K[i][j] = 0.0;

            for (int k = 0; k < 3; k++) {
                K[i][j] += P_H_transpose[i][k] * S_inverse[k][j];
            }
        }
    }
}

// EKF STAGE 10
// Quaternion correction
void quaternion_correction_stage_10(double *q_w, double *q_x, double *q_y, double *q_z, double K[4][3], double residual_x, double residual_y, double residual_z)
{
    double correction_w = K[0][0] * residual_x + K[0][1] * residual_y + K[0][2] * residual_z;
    double correction_x = K[1][0] * residual_x + K[1][1] * residual_y + K[1][2] * residual_z;
    double correction_y = K[2][0] * residual_x + K[2][1] * residual_y + K[2][2] * residual_z;
    double correction_z = K[3][0] * residual_x + K[3][1] * residual_y + K[3][2] * residual_z;

    *q_w += correction_w;
    *q_x += correction_x;
    *q_y += correction_y;
    *q_z += correction_z;

    double q_magnitude = sqrt(*q_w * *q_w + *q_x * *q_x + *q_y * *q_y + *q_z * *q_z);

    if (q_magnitude < 1e-12) {
        return;
    }

    *q_w /= q_magnitude;
    *q_x /= q_magnitude;
    *q_y /= q_magnitude;
    *q_z /= q_magnitude;
}

// EKF STAGE 11
// Covariance update
void covariance_update_stage_11(double P[4][4], double K[4][3], double H[3][4])
{
    double K_H[4][4] = {{0.0}};
    double I_minus_K_H[4][4] = {{0.0}};
    double P_new[4][4] = {{0.0}};

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 3; k++) {
                K_H[i][j] += K[i][k] * H[k][j];
            }
        }
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (i == j) {
                I_minus_K_H[i][j] = 1.0 - K_H[i][j];
            } else {
                I_minus_K_H[i][j] = -K_H[i][j];
            }
        }
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                P_new[i][j] += I_minus_K_H[i][k] * P[k][j];
            }
        }
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P[i][j] = P_new[i][j];
        }
    }
}

// COMPLETE EKF UPDATE
void ekf_update(double *q_w, double *q_x, double *q_y, double *q_z, double P[4][4], double Q[4][4], double R[3][3], double H[3][4], double F[4][4], double S[3][3], double K[4][3], double g_x, double g_y, double g_z, double a_x, double a_y, double a_z, double dt)
{
    double predicted_g_x = 0.0;
    double predicted_g_y = 0.0;
    double predicted_g_z = 0.0;

    double z_x = 0.0;
    double z_y = 0.0;
    double z_z = 0.0;

    double residual_x = 0.0;
    double residual_y = 0.0;
    double residual_z = 0.0;

    quaternion_prediction_stage_1(q_w, q_x, q_y, q_z, g_x, g_y, g_z, dt);
    measurement_prediction_stage_2(*q_w, *q_x, *q_y, *q_z, &predicted_g_x, &predicted_g_y, &predicted_g_z);
    acceleration_measurement_stage_3(a_x, a_y, a_z, &z_x, &z_y, &z_z);
    residual_stage_4(z_x, z_y, z_z, predicted_g_x, predicted_g_y, predicted_g_z, &residual_x, &residual_y, &residual_z);
    measurement_jacobian_h_stage_5(*q_w, *q_x, *q_y, *q_z, H);
    state_transition_jacobian_f_stage_6(g_x, g_y, g_z, dt, F);
    covariance_prediction_stage_6(P, F, Q);
    measurement_noise_stage_7(R);
    innovation_covariance_stage_8(H, P, R, S);
    kalman_gain_stage_9(P, H, S, K);
    quaternion_correction_stage_10(q_w, q_x, q_y, q_z, K, residual_x, residual_y, residual_z);
    covariance_update_stage_11(P, K, H);
}

double calculate_radar_angle(double q_w, double q_x, double q_y, double q_z)
{
    double predicted_g_z = -(q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z);

    predicted_g_z = fmax(-1.0, fmin(1.0, predicted_g_z));

    return acos(-predicted_g_z) * 180.0 / M_PI;
}