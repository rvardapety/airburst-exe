#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "airburst.h"
// #include "esp32_cam.h"
#include "include/functions.h"

double distance_threshold = 1.0;

double q_w = 1.0;
double q_x = 0.0;
double q_y = 0.0;
double q_z = 0.0;

double gyro_bias_x = 0.0;
double gyro_bias_y = 0.0;
double gyro_bias_z = 0.0;

double P[4][4] = {
    {0.01, 0.0, 0.0, 0.0},
    {0.0, 0.01, 0.0, 0.0},
    {0.0, 0.0, 0.01, 0.0},
    {0.0, 0.0, 0.0, 0.01}
};

double Q[4][4] = {
    {0.0001, 0.0, 0.0, 0.0},
    {0.0, 0.0001, 0.0, 0.0},
    {0.0, 0.0, 0.0001, 0.0},
    {0.0, 0.0, 0.0, 0.0001}
};

double R[3][3] = {{0.0}};
double H[3][4] = {{0.0}};
double F[4][4] = {{0.0}};
double S[3][3] = {{0.0}};
double K[4][3] = {{0.0}};

int main(void)
{
    bool state = true;

    if (!imu_i2c_init()) {
        printf("ERROR: imu_i2c_init() failed\n");
        imu_i2c_deinit();
        return EXIT_FAILURE;
    }

    if (!airburst_init()) {
        printf("ERROR: airburst_init() failed\n");
        imu_i2c_deinit();
        return EXIT_FAILURE;
    }

    // esp32_cam_init();

    // On failure, airburst_distance_init() already deactivates RSS itself
    if (!airburst_distance_init()) {
        printf("ERROR: airburst_distance_init() failed\n");
        imu_i2c_deinit();
        return EXIT_FAILURE;
    }

    if (!calculate_gyro_bias(&gyro_bias_x, &gyro_bias_y, &gyro_bias_z)) {
        printf("ERROR: gyro bias calibration failed (no IMU data)\n");
        airburst_distance_destroy();
        airburst_destroy();
        imu_i2c_deinit();
        return EXIT_FAILURE;
    }

    printf("Gyro bias: X=%.6f Y=%.6f Z=%.6f rad/s\n", gyro_bias_x, gyro_bias_y, gyro_bias_z);

    gyro_value_t gyro = {0.0f, 0.0f, 0.0f};
    accel_value_t accel = {0.0f, 0.0f, 0.0f};

    struct timespec previous_time;
    struct timespec current_time;

    clock_gettime(CLOCK_MONOTONIC, &previous_time);

    while (state) {
        // Skip this cycle on a failed read; previous_time is not advanced,
        // so the next dt covers the skipped interval
        if (!imu_get_gyro_data(&gyro) || !imu_get_accel_data(&accel)) {
            printf("WARNING: IMU read failed, skipping cycle\n");
            usleep(200000);
            continue;
        }

        clock_gettime(CLOCK_MONOTONIC, &current_time);

        double dt = calculate_dt(&previous_time, &current_time);

        printf("dt = %.6f s\n", dt);

        gyro.gyro_x = (float)((double)gyro.gyro_x - gyro_bias_x);
        gyro.gyro_y = (float)((double)gyro.gyro_y - gyro_bias_y);
        gyro.gyro_z  = (float)((double)gyro.gyro_z - gyro_bias_z);

        double a_x = (double)accel.accel_x - 0.023;
        double a_y = (double)accel.accel_y - 0.051;
        double a_z = (double)accel.accel_z;

        ekf_update(&q_w, &q_x, &q_y, &q_z, P, Q, R, H, F, S, K, (double)gyro.gyro_x, (double)gyro.gyro_y, (double)gyro.gyro_z, a_x, a_y, a_z, dt);

        //radar angle calculation
        double radar_angle = calculate_radar_angle(q_w, q_x, q_y, q_z);
        printf("Radar angle: %.2f degrees\n", radar_angle);

        switch (airburst_fall_state(accel, gyro, radar_angle)) {
        case UNSTABLE:
            airburst_destabilized();
            printf("STATE: UNSTABLE\n");
            break;

        case STABLE:
            airburst_stabilized();
            printf("STATE: STABLE\n");
            break;

        case MEASURING: {
                float raw_distance = airburst_get_distance_m();

                // Sensor error or no target: never trigger on an invalid reading
                if (raw_distance < 0.0f) {   // AIRBURST_NO_DISTANCE
                    printf("STATE: MEASURING | Distance: no target\n");
                    break;
                }

                double distance = (double)raw_distance;

                printf("STATE: MEASURING | Distance: %.2f m\n", distance);

                if (distance > 0.0 && distance <= distance_threshold) {
                    airburst_led_on();
                    printf("LED ON!\n");
                    state = false;
                }
                break;
        }
        default:
            break;
        }

        usleep(200000);
    }
    airburst_distance_destroy();
    airburst_destroy();
    imu_i2c_deinit();

    return EXIT_SUCCESS;
}