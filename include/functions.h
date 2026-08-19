#ifndef AIRBURST_EXE_FUNCTIONS_H
#define AIRBURST_EXE_FUNCTIONS_H

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include "../subprojects/imu_lib/include/imu.h"

#define STABLE_ACCEL_MIN_G 0.8
#define STABLE_ACCEL_MAX_G 1.2
#define ACCEL_THRESHOLD_VALUE_G 0.2
#define STABLE_MEASUREMENT_COUNT 10
#define RADAR_ANGLE_THRESHOLD_DEG 5.0
#define STABLE_GYRO_THRESHOLD_DPS 15.0

typedef enum {
    STABLE,
    FREE_FALL,
    WAITING_FOR_STABILITY,
} airbust_state_t;

double calculate_dt (struct timespec *previous_time, struct timespec *current_time);
airbust_state_t airburst_fall_state (accel_value_t accel_value, gyro_value_t gyro_value);
void quaternion_prediction_stage_1 (double *q_w, double *q_x, double *q_y, double *q_z, double g_x, double g_y, double g_z, double dt);
void measurement_prediction_stage_2 (double q_w, double q_x, double q_y, double q_z, double *predicted_g_x, double *predicted_g_y, double *predicted_g_z);
void acceleration_measurement_stage_3 (double a_x, double a_y, double a_z, double *z_x, double *z_y, double *z_z);
void residual_stage_4 (double z_x, double z_y, double z_z, double predicted_g_x, double predicted_g_y, double predicted_g_z, double *residual_x, double *residual_y, double *residual_z);

#endif //AIRBURST_EXE_FUNCTIONS_H
