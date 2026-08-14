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


