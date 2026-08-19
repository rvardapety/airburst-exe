#include "imu.h"
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include "include/functions.h"

int main (void) {

    bool state = true;

    imu_i2c_init();

    gyro_value_t gyro;
    accel_value_t accel;

    struct timespec previous_time, current_time;
    clock_gettime(CLOCK_MONOTONIC, &previous_time);

    while (state) {

        clock_gettime(CLOCK_MONOTONIC, &current_time);

        double dt = calculate_dt(&previous_time, &current_time);

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

        // EKF STAGE 2: Predicted measurement

        measurement_prediction_stage_2(q_w, q_x, q_y, q_z, &predicted_g_x, &predicted_g_y, &predicted_g_z);

        // EKF STAGE 3: Accelerometer measurement

        acceleration_measurement_stage_3(a_x, a_y, a_z, &z_x, &z_y, &z_z);

        // EKF STAGE 4: Residual

        residual_stage_4(z_x, z_y, z_z, predicted_g_x, predicted_g_y, predicted_g_z, &residual_x, &residual_y, &residual_z);

        // EKF STAGE 5: Measurement Jacobian H

        measurement_jacobian_h_stage_5(q_w, q_x, q_y, q_z, H);

        // EKF STAGE 6: State transition Jacobian F

        state_transition_jacobian_f_stage_6(g_x, g_y, g_z, dt, F);

        // EKF STAGE 6: Covariance prediction

        covariance_prediction_stage_6(P, F, Q);

        // EKF STAGE 7: Measurement noise R

        measurement_noise_stage_7(R);

        // EKF STAGE 8: Innovation covariance S

        innovation_covariance_stage_8(H, P, R, S);

        // EKF STAGE 9: Kalman gain K

        kalman_gain_stage_9(P, H, S, K);

        // EKF STAGE 10: Quaternion correction

        quaternion_correction_stage_10(&q_w, &q_x, &q_y, &q_z, K, residual_x, residual_y, residual_z);

        // EKF STAGE 11: Covariance update

        covariance_update_stage_11(P, K, H);

        printf("Acceleration X: %.3f g | Y: %.3f g | Z: %.3f g\n", a_x, a_y, a_z);

        printf("Gyro X: %.6f rad/s | Y: %.6f rad/s | Z: %.6f rad/s\n", g_x, g_y, g_z);

        printf("Quaternion: qw=%.6f qx=%.6f qy=%.6f qz=%.6f\n", q_w, q_x, q_y, q_z);

        printf("Predicted gravity: X=%.6f Y=%.6f Z=%.6f\n", predicted_g_x, predicted_g_y, predicted_g_z);

        printf("Measurement: X=%.6f Y=%.6f Z=%.6f\n", z_x, z_y, z_z);

        printf("Residual: X=%.6f Y=%.6f Z=%.6f\n", residual_x, residual_y, residual_z);

        usleep(200000);
    }

    // imu_i2c_deinit();

    return 0;
}