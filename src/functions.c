#include "../include/functions.h"

uint8_t count = 0;

airbust_state_t airburst_fall_state (accel_value_t accel_value, gyro_value_t gyro_value) {

    imu_get_gyro_data(&gyro_value);
    imu_get_accel_data(&accel_value);

    double gx = (double)gyro_value.gyro_x;
    double gy = (double)gyro_value.gyro_y;
    double gz = (double)gyro_value.gyro_z;

    double ax = (double)accel_value.accel_x;
    double ay = (double)accel_value.accel_y;
    double az = (double)accel_value.accel_z;

    // 1) Add Kalman filter here, and check the filtered data afterward.
    // 2) Create a buffer and fill it with filtered data.

    double gyro_magnitude = sqrt(gx * gx + gy * gy + gz * gz);
    double accel_magnitude = sqrt(ax * ax + ay * ay + az * az);

    if (accel_magnitude < ACCEL_THRESHOLD_VALUE_G) {
        return FREE_FALL;
    }

    if (accel_magnitude >= STABLE_ACCEL_MIN_G
        && accel_magnitude < STABLE_ACCEL_MAX_G
        && gyro_magnitude < STABLE_GYRO_THRESHOLD_DPS) {

        count++;

        if (count == STABLE_MEASUREMENT_COUNT) {
            return STABLE;
        }
        return WAITING_FOR_STABILITY;
    }

    count = 0;
    return WAITING_FOR_STABILITY;
}

double calculate_dt(struct timespec *previous_time, struct timespec *current_time)
{
    double dt = (double)(current_time->tv_sec - previous_time->tv_sec)
              + (double)(current_time->tv_nsec - previous_time->tv_nsec) / 1e9;

    *previous_time = *current_time;

    return dt;
}

void quaternion_prediction_stage_1 (double *q_w, double *q_x, double *q_y, double *q_z, double g_x, double g_y, double g_z, double dt) {
    double alpha_w = -*q_x * g_x - *q_y * g_y - *q_z * g_z;
    double alpha_x =  *q_w * g_x + *q_y * g_z - *q_z * g_y;
    double alpha_y =  *q_w * g_y - *q_x * g_z + *q_z * g_x;
    double alpha_z =  *q_w * g_z + *q_x * g_y - *q_y * g_x;

    double qw_pred = *q_w + 0.5 * alpha_w * dt;
    double qx_pred = *q_x + 0.5 * alpha_x * dt;
    double qy_pred = *q_y + 0.5 * alpha_y * dt;
    double qz_pred = *q_z + 0.5 * alpha_z * dt;

    double q_magnitude = sqrt(qw_pred * qw_pred + qx_pred * qx_pred + qy_pred * qy_pred + qz_pred * qz_pred);

    *q_w = qw_pred / q_magnitude;
    *q_x = qx_pred / q_magnitude;
    *q_y = qy_pred / q_magnitude;
    *q_z = qz_pred / q_magnitude;
}

void measurement_prediction_stage_2 (double q_w, double q_x, double q_y, double q_z, double *predicted_g_x, double *predicted_g_y, double *predicted_g_z) {
    *predicted_g_x = 2.0 * (q_x * q_z + q_w * q_y);
    *predicted_g_y = 2.0 * (q_y * q_z - q_w * q_x);
    *predicted_g_z = -(q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z);
}

void acceleration_measurement_stage_3 (double a_x, double a_y, double a_z, double *z_x, double *z_y, double *z_z) {
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

void residual_stage_4 (double z_x, double z_y, double z_z,
                       double predicted_g_x, double predicted_g_y, double predicted_g_z,
                       double *residual_x, double *residual_y, double *residual_z) {

    *residual_x = z_x - predicted_g_x;
    *residual_y = z_y - predicted_g_y;
    *residual_z = z_z - predicted_g_z;
}

void measurement_jacobian_h_stage_5(double q_w, double q_x, double q_y, double q_z, double jacobian_h[3][4]){
    jacobian_h[0][0] =  2.0 * q_y;
    jacobian_h[0][1] =  2.0 * q_z;
    jacobian_h[0][2] =  2.0 * q_w;
    jacobian_h[0][3] =  2.0 * q_x;

    jacobian_h[1][0] = -2.0 * q_x;
    jacobian_h[1][1] = -2.0 * q_w;
    jacobian_h[1][2] =  2.0 * q_z;
    jacobian_h[1][3] =  2.0 * q_y;

    jacobian_h[2][0] = -2.0 * q_w;
    jacobian_h[2][1] =  2.0 * q_x;
    jacobian_h[2][2] =  2.0 * q_y;
    jacobian_h[2][3] = -2.0 * q_z;
}

void state_transition_jacobian_f_stage_6 (double g_x, double g_y, double g_z, double dt, double F[4][4]) {

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


void covariance_prediction_stage_6 (double P[4][4], double F[4][4], double Q[4][4]) {

    double FP[4][4] = {{0.0}};
    double F_transpose[4][4] = {{0.0}};
    double P_predicted[4][4] = {{0.0}};

    // F transpose

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            F_transpose[i][j] = F[j][i];
        }
    }

    // F * P

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                FP[i][j] += F[i][k] * P[k][j];
            }
        }
    }

    // F * P * F^T

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                P_predicted[i][j] += FP[i][k] * F_transpose[k][j];
            }
        }
    }

    // F * P * F^T + Q

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P[i][j] = P_predicted[i][j] + Q[i][j];
        }
    }
}


void measurement_noise_stage_7 (double R[3][3]) {

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


void innovation_covariance_stage_8 (double jacobian_h[3][4], double P[4][4],
                                    double R[3][3], double S[3][3]) {

    double H_P[3][4] = {{0.0}};
    double H_transpose[4][3] = {{0.0}};

    // H transpose

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            H_transpose[j][i] = jacobian_h[i][j];
        }
    }

    // H * P

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                H_P[i][j] += jacobian_h[i][k] * P[k][j];
            }
        }
    }

    // H * P * H^T + R

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {

            S[i][j] = R[i][j];

            for (int k = 0; k < 4; k++) {
                S[i][j] += H_P[i][k] * H_transpose[k][j];
            }
        }
    }
}


void kalman_gain_stage_9 (double P[4][4], double jacobian_h[3][4],
                          double S[3][3], double K[4][3]) {

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

    // H transpose

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            H_transpose[j][i] = jacobian_h[i][j];
        }
    }

    // S inverse

    S_inverse[0][0] = (S[1][1] * S[2][2] - S[1][2] * S[2][1]) / determinant;
    S_inverse[0][1] = (S[0][2] * S[2][1] - S[0][1] * S[2][2]) / determinant;
    S_inverse[0][2] = (S[0][1] * S[1][2] - S[0][2] * S[1][1]) / determinant;

    S_inverse[1][0] = (S[1][2] * S[2][0] - S[1][0] * S[2][2]) / determinant;
    S_inverse[1][1] = (S[0][0] * S[2][2] - S[0][2] * S[2][0]) / determinant;
    S_inverse[1][2] = (S[0][2] * S[1][0] - S[0][0] * S[1][2]) / determinant;

    S_inverse[2][0] = (S[1][0] * S[2][1] - S[1][1] * S[2][0]) / determinant;
    S_inverse[2][1] = (S[0][1] * S[2][0] - S[0][0] * S[2][1]) / determinant;
    S_inverse[2][2] = (S[0][0] * S[1][1] - S[0][1] * S[1][0]) / determinant;

    // P * H^T

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 4; k++) {
                P_H_transpose[i][j] += P[i][k] * H_transpose[k][j];
            }
        }
    }

    // P * H^T * S^-1

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {

            K[i][j] = 0.0;

            for (int k = 0; k < 3; k++) {
                K[i][j] += P_H_transpose[i][k] * S_inverse[k][j];
            }
        }
    }
}


void quaternion_correction_stage_10 (double *q_w, double *q_x, double *q_y, double *q_z,
                                     double K[4][3],
                                     double residual_x, double residual_y, double residual_z) {

    double correction_w =
          K[0][0] * residual_x
        + K[0][1] * residual_y
        + K[0][2] * residual_z;

    double correction_x =
          K[1][0] * residual_x
        + K[1][1] * residual_y
        + K[1][2] * residual_z;

    double correction_y =
          K[2][0] * residual_x
        + K[2][1] * residual_y
        + K[2][2] * residual_z;

    double correction_z =
          K[3][0] * residual_x
        + K[3][1] * residual_y
        + K[3][2] * residual_z;

    *q_w += correction_w;
    *q_x += correction_x;
    *q_y += correction_y;
    *q_z += correction_z;

    double q_magnitude =
        sqrt(*q_w * *q_w
           + *q_x * *q_x
           + *q_y * *q_y
           + *q_z * *q_z);

    *q_w /= q_magnitude;
    *q_x /= q_magnitude;
    *q_y /= q_magnitude;
    *q_z /= q_magnitude;
}


void covariance_update_stage_11 (double P[4][4], double K[4][3], double jacobian_h[3][4]) {

    double K_H[4][4] = {{0.0}};
    double I_minus_K_H[4][4] = {{0.0}};
    double P_new[4][4] = {{0.0}};

    // K * H

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 3; k++) {
                K_H[i][j] += K[i][k] * jacobian_h[k][j];
            }
        }
    }

    // I - K * H

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {

            if (i == j) {
                I_minus_K_H[i][j] = 1.0 - K_H[i][j];
            } else {
                I_minus_K_H[i][j] = -K_H[i][j];
            }
        }
    }

    // (I - K * H) * P

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                P_new[i][j] += I_minus_K_H[i][k] * P[k][j];
            }
        }
    }

    // Copy back to P

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P[i][j] = P_new[i][j];
        }
    }
}