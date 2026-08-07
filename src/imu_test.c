#include <stdio.h>
#include <unistd.h>
#include "imu.h"

int main (void) {

    bool state = true;

    imu_i2c_init();
    accel_value_t accel;
    gyro_value_t gyro;

    while (state) {
        imu_get_accel_data(&accel);
        imu_get_gyro_data(&gyro);
        printf("Acceleration X: %.3f g | Y: %.3f g | Z: %.3f g\n", (double)accel.accel_x, (double)accel.accel_y, (double)accel.accel_z);

        printf("Gyro X: %.2f deg/s | Y: %.2f deg/s | Z: %.2f deg/s\n",(double)gyro.gyro_x, (double)gyro.gyro_y, (double)gyro.gyro_z);

        usleep(200000);
    }

    // imu_i2c_deinit();
    return 0;
}


