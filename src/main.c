#include <stdio.h>
#include <unistd.h>

#include "airburst.h"
#include "imu.h"

int main (void) {

    bool state = true;
    // float target_height = 0.04;

    imu_i2c_init();
    // airburst_init();
    // airburst_distance_init();
    accel_value_t accel;
    gyro_value_t gyro;

    while (state) {
        imu_get_accel_data(&accel);
        imu_get_gyro_data(&gyro);
        printf("Acceleration X: %5d | Y: %5d | Z: %5d\n", accel.accel_x, accel.accel_y, accel.accel_z);
        printf("Gyro X: %5d | Y: %5d | Z: %5d\n", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

        usleep(200000);
    }

    // airburst_distance_destroy();
    // airburst_destroy();
    imu_i2c_deinit();

    return 0;
}
//
// float distance = airburst_get_distance_m();
// printf("distance m is %f\n", (double)distance);
// if (distance < target_height) {
//     airburst_bust_bomb();
//     printf("boom at %f\n", (double)distance);
//     state = false;
// }
