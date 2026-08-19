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

double calculate_dt(struct timespec *previous_time,
                    struct timespec *current_time)
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

    *z_x = a_x / accel_magnitude;
    *z_y = a_y / accel_magnitude;
    *z_z = a_z / accel_magnitude;
}
