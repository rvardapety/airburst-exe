#ifndef AIRBURST_EXE_FUNCTIONS_H
#define AIRBURST_EXE_FUNCTIONS_H

#include <stdbool.h>
#include <time.h>

#include "imu.h"

// Stability check (evaluated by airburst_fall_state() at the control rate)
#define STABLE_ACCEL_MIN_G 0.8
#define STABLE_ACCEL_MAX_G 1.2
#define STABLE_MEASUREMENT_COUNT 10
#define RADAR_ANGLE_THRESHOLD_DEG 5.0
#define STABLE_GYRO_THRESHOLD_RAD 0.261799

// Accelerometer offsets (g), measured for this IMU and subtracted from every sample
#define ACCEL_OFFSET_X_G 0.023
#define ACCEL_OFFSET_Y_G 0.051
#define ACCEL_OFFSET_Z_G 0.0

// Gyro bias calibration: number of samples (10 ms apart) and the largest per-axis
// standard deviation (rad/s) accepted as "device held still"
#define GYRO_CALIBRATION_SAMPLES 100
#define GYRO_CALIBRATION_MAX_STD_RAD 0.02

// EKF accelerometer gate: the accelerometer is only a gravity reference when
// |a| is close to 1 g. Outside 1 g +/- ACCEL_GATE_G the EKF uses the gyro only.
#define ACCEL_GATE_G 0.1

// EKF tuning: initial covariance, process noise and accelerometer noise (diagonals)
#define EKF_P0 0.01
#define EKF_Q  0.0001
#define EKF_R  0.01

// IMU mounting convention (used by the EKF measurement model).
// The value is the accelerometer Z reading, in g, when the radar is level (0 deg):
//   -1.0 -> IMU Z axis points down when level (accel_z reads about -1 g)  [previous behavior]
//   +1.0 -> IMU Z axis points up when level   (accel_z reads about +1 g)
// If the radar angle reads about 180 deg while level, flip this sign.
#define IMU_GRAVITY_SIGN (-1.0)

#define RAD_TO_DEG (180.0 / 3.14159265358979323846)

typedef enum {
    UNSTABLE,
    STABLE,
    MEASURING
} airburst_state_t;

// EKF state: orientation quaternion q = [w, x, y, z] (rotates body -> world)
// and its 4x4 covariance P, plus the constant noise matrices Q and R.
typedef struct {
    double q[4];
    double P[4][4];
    double Q[4][4];
    double R[3][3];
} ekf_t;

airburst_state_t airburst_fall_state(double accel_magnitude_g, double gyro_magnitude_rad, double radar_angle_deg);

double calculate_dt(struct timespec *previous_time, const struct timespec *current_time);

// Averages GYRO_CALIBRATION_SAMPLES gyro readings into bias[x, y, z] (rad/s).
// max_std_rad receives the largest per-axis standard deviation, i.e. how much the
// device moved; compare it with GYRO_CALIBRATION_MAX_STD_RAD.
// Returns false if no gyro sample could be read.
bool calculate_gyro_bias(double bias[3], double *max_std_rad);

// Sets q to identity and P, Q, R to their EKF_* diagonals.
void ekf_init(ekf_t *ekf);

// One EKF step: gyro prediction (always) and accelerometer correction (when |a|
// is within 1 g +/- ACCEL_GATE_G). gyro in rad/s, accel in g, dt in s.
// Returns true if the accelerometer correction was applied.
bool ekf_update(ekf_t *ekf, const double gyro[3], const double accel[3], double dt);

// Tilt between the body Z axis and the world Z axis, in degrees
double ekf_radar_angle_deg(const ekf_t *ekf);

#endif // AIRBURST_EXE_FUNCTIONS_H
