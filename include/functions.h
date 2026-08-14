#ifndef AIRBURST_EXE_FUNCTIONS_H
#define AIRBURST_EXE_FUNCTIONS_H

#include <math.h>
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

airbust_state_t airburst_fall_state (accel_value_t accel_value, gyro_value_t gyro_value);


#endif //AIRBURST_EXE_FUNCTIONS_H
