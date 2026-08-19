#include "imu.h"
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include "include/functions.h"

double q_w = 1.0;
double q_x = 0.0;
double q_y = 0.0;
double q_z = 0.0;

double z_x = 0.0;
double z_y = 0.0;
double z_z = 0.0;

double predicted_g_x = 0.0;
double predicted_g_y = 0.0;
double predicted_g_z = 0.0;

double residual_x = 0.0;
double residual_y = 0.0;
double residual_z = 0.0;

double P[4][4] = {
    {0.01, 0.0,  0.0,  0.0},
    {0.0,  0.01, 0.0,  0.0},
    {0.0,  0.0,  0.01, 0.0},
    {0.0,  0.0,  0.0,  0.01}
};

double Q[4][4] = {
    {0.000001, 0.0,      0.0,      0.0},
    {0.0,      0.000001, 0.0,      0.0},
    {0.0,      0.0,      0.000001, 0.0},
    {0.0,      0.0,      0.0,      0.000001}
};

int main (void) {

    bool state = true;

    imu_i2c_init();
    gyro_value_t gyro;
    accel_value_t accel;

    struct timespec previous_time, current_time;
    clock_gettime(CLOCK_MONOTONIC, &previous_time);



    while (state) {
        double dt = calculate_dt(&previous_time,&current_time);

        printf("dt = %.6f s\n", dt);

        imu_get_gyro_data(&gyro);
        imu_get_accel_data(&accel);

        double g_x = (double)gyro.gyro_x;
        double g_y = (double)gyro.gyro_y;
        double g_z = (double)gyro.gyro_z;

        double a_x = (double)accel.accel_x - 0.023;
        double a_y = (double)accel.accel_y - 0.051;
        double a_z = (double)accel.accel_z;

        // EKF STAGE 1: Quaternion prediction
        quaternion_prediction_stage_1(&q_w, &q_x, &q_y, &q_z, g_x, g_y, g_z, dt);

        //EKF STAGE 2: Predicted measurement
        measurement_prediction_stage_2(q_w, q_x, q_y, q_z, &predicted_g_x, &predicted_g_y, &predicted_g_z);


        // EKF STAGE 3: Accelerometer measurement
        acceleration_measurement_stage_3 (a_x, a_y, a_z, &z_x, &z_y, &z_z);

        //EKF STAGE 4: Residual
        residual_stage_4 (z_x, z_y, z_z, predicted_g_x, predicted_g_y, predicted_g_z, &residual_x, &residual_y, &residual_z);

        //EKF STAGE 5: Measurement Jacobian H

        double jacobian_h [3][4] = {
            {2.0 * q_y,  2.0 * q_z,  2.0 * q_w,  2.0 * q_x},
            {-2.0 * q_x, -2.0 * q_w, 2.0 * q_z,  2.0 * q_y},
            {-2.0 * q_w, 2.0 * q_x,  2.0 * q_y, -2.0 * q_z}
        };

        //EKF STAGE 6: Jacobian F

        double F[4][4] = {
            {1.0, -0.5 * g_x * dt, -0.5 * g_y * dt, -0.5 * g_z * dt},
            {0.5 * g_x * dt, 1.0, 0.5 * g_z * dt, -0.5 * g_y * dt},
            {0.5 * g_y * dt, -0.5 * g_z * dt, 1.0, 0.5 * g_x * dt},
            {0.5 * g_z * dt, 0.5 * g_y * dt, -0.5 * g_x * dt, 1.0}
        };








        usleep(200000);
    }

    // imu_i2c_deinit();
    return 0;
}
