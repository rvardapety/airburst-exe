#include "imu.h"
#include <math.h>
#include <stdio.h>
#include <unistd.h>

int main (void) {

    bool state = true;

    imu_i2c_init();
    accel_value_t accel;
    gyro_value_t gyro;

    while (state) {
        imu_get_gyro_data(&gyro);
        imu_get_accel_data(&accel);

        double ax = (double)accel.accel_x - 0.023;
        double ay = (double)accel.accel_y - 0.051;
        double az = (double)accel.accel_z;

        double gx = (double)gyro.gyro_x;
        double gy = (double)gyro.gyro_y;
        double gz = (double)gyro.gyro_z;

        double accel_magnitude = sqrt(ax * ax + ay * ay + az * az);
        double ratio = -az / accel_magnitude;
        double angle = acos(ratio);

        double gyro_magnitude = sqrt(gx * gx + gy * gy + gz * gz);

        printf("Acceleration X: %.3f g | Y: %.3f g | Z: %.3f g\n", (double)accel.accel_x - 0.023, (double)accel.accel_y - 0.051, (double)accel.accel_z);
        printf("Gyro X: %.2f deg/s | Y: %.2f deg/s | Z: %.2f deg/s\n",(double)gyro.gyro_x, (double)gyro.gyro_y, (double)gyro.gyro_z);
        printf("angle=%.6f rad | %.2f deg\n", angle, angle * 180.0 / M_PI);

        usleep(200000);
    }

    // imu_i2c_deinit();
    return 0;
}
