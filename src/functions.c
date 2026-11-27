#define _POSIX_C_SOURCE 200809L

#include "functions.h"

#include <stdint.h>
#include <math.h>
#include <time.h>

static uint8_t count = 0;
static airburst_state_t current_state = UNSTABLE;

static void sleep_ms(long ms)
{
    struct timespec duration = { .tv_sec = ms / 1000, .tv_nsec = (ms % 1000) * 1000000L };

    nanosleep(&duration, NULL);
}

// =====================================================================
// AIRBURST FALL / STABILITY STATE
// =====================================================================
// Called once per control step with bias/offset-corrected magnitudes.
//   any state  --(conditions not met)-------------------------> UNSTABLE (count reset)
//   UNSTABLE   --(conditions met for STABLE_MEASUREMENT_COUNT)--> STABLE
//   STABLE     --(conditions still met, next step)-------------> MEASURING
//   MEASURING  --(conditions still met)------------------------> MEASURING
airburst_state_t airburst_fall_state(double accel_magnitude_g, double gyro_magnitude_rad, double radar_angle_deg)
{
    bool conditions_met =
        accel_magnitude_g >= STABLE_ACCEL_MIN_G &&
        accel_magnitude_g < STABLE_ACCEL_MAX_G &&
        gyro_magnitude_rad < STABLE_GYRO_THRESHOLD_RAD &&
        radar_angle_deg <= RADAR_ANGLE_THRESHOLD_DEG;

    if (!conditions_met) {
        count = 0;
        current_state = UNSTABLE;
        return current_state;
    }

    // Saturate: an unbounded uint8_t would wrap to 0 after 256 stable samples
    if (count < STABLE_MEASUREMENT_COUNT) {
        count++;
    }

    switch (current_state) {
    case UNSTABLE:
        if (count >= STABLE_MEASUREMENT_COUNT) {
            current_state = STABLE;
        }
        break;

    case STABLE:
        current_state = MEASURING;
        break;

    case MEASURING:
    default:
        break;
    }

    return current_state;
}

// =====================================================================
// GYROSCOPE BIAS CALCULATION
// =====================================================================
bool calculate_gyro_bias(double bias[3], double *max_std_rad)
{
    gyro_value_t gyro = {0.0f, 0.0f, 0.0f};

    double sum[3] = {0.0, 0.0, 0.0};
    double sum_squares[3] = {0.0, 0.0, 0.0};
    int valid_count = 0;

    for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
        if (imu_get_gyro_data(&gyro)) {
            double sample[3] = {(double)gyro.gyro_x, (double)gyro.gyro_y, (double)gyro.gyro_z};

            for (int axis = 0; axis < 3; axis++) {
                sum[axis] += sample[axis];
                sum_squares[axis] += sample[axis] * sample[axis];
            }
            valid_count++;
        }

        sleep_ms(10);
    }

    *max_std_rad = 0.0;

    if (valid_count == 0) {
        bias[0] = bias[1] = bias[2] = 0.0;
        return false;
    }

    for (int axis = 0; axis < 3; axis++) {
        double mean = sum[axis] / valid_count;
        double variance = sum_squares[axis] / valid_count - mean * mean;
        double std_dev = sqrt(fmax(variance, 0.0));

        bias[axis] = mean;
        *max_std_rad = fmax(*max_std_rad, std_dev);
    }

    return true;
}

// =====================================================================
// DT CALCULATION
// =====================================================================
double calculate_dt(struct timespec *previous_time, const struct timespec *current_time)
{
    double dt = (double)(current_time->tv_sec - previous_time->tv_sec) + (double)(current_time->tv_nsec - previous_time->tv_nsec) / 1e9;

    *previous_time = *current_time;

    return dt;
}

// =====================================================================
// SMALL MATRIX HELPERS (row-major, sizes given explicitly)
// =====================================================================

// C[n][p] = A[n][m] * B[m][p]
static void mat_mul(int n, int m, int p, double A[n][m], double B[m][p], double C[n][p])
{
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < p; j++) {
            C[i][j] = 0.0;
            for (int k = 0; k < m; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// T[m][n] = A[n][m]^T
static void mat_transpose(int n, int m, double A[n][m], double T[m][n])
{
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            T[j][i] = A[i][j];
        }
    }
}

static void normalize_quaternion(double q[4])
{
    double magnitude = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

    if (magnitude < 1e-12) {
        return;
    }

    for (int i = 0; i < 4; i++) {
        q[i] /= magnitude;
    }
}

// =====================================================================
// EKF - PREDICTION (gyroscope)
// =====================================================================

// EKF STAGE 1 - Quaternion prediction: q <- normalize(q + 0.5 * q (x) [0, g] * dt)
static void predict_quaternion_stage_1(double q[4], const double g[3], double dt)
{
    double w = q[0], x = q[1], y = q[2], z = q[3];

    double q_dot[4] = {
        -x * g[0] - y * g[1] - z * g[2],
         w * g[0] + y * g[2] - z * g[1],
         w * g[1] - x * g[2] + z * g[0],
         w * g[2] + x * g[1] - y * g[0]
    };

    for (int i = 0; i < 4; i++) {
        q[i] += 0.5 * q_dot[i] * dt;
    }

    normalize_quaternion(q);
}

// EKF STAGE 2 - State transition Jacobian F = d q_predicted / d q (matches stage 1)
static void state_jacobian_f_stage_2(const double g[3], double dt, double F[4][4])
{
    double hx = 0.5 * g[0] * dt;
    double hy = 0.5 * g[1] * dt;
    double hz = 0.5 * g[2] * dt;

    double jacobian[4][4] = {
        {1.0, -hx, -hy, -hz},
        {hx,  1.0,  hz, -hy},
        {hy,  -hz, 1.0,  hx},
        {hz,   hy, -hx, 1.0}
    };

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            F[i][j] = jacobian[i][j];
        }
    }
}

// EKF STAGE 3 - Covariance prediction: P <- F P F^T + Q
static void predict_covariance_stage_3(double P[4][4], double F[4][4], double Q[4][4])
{
    double FP[4][4];
    double F_transpose[4][4];
    double FPFt[4][4];

    mat_transpose(4, 4, F, F_transpose);
    mat_mul(4, 4, 4, F, P, FP);
    mat_mul(4, 4, 4, FP, F_transpose, FPFt);

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P[i][j] = FPFt[i][j] + Q[i][j];
        }
    }
}

// =====================================================================
// EKF - CORRECTION (accelerometer)
// =====================================================================

// EKF STAGE 4 - Predicted measurement h(q): gravity direction in the BODY frame.
// q rotates body -> world, so h(q) = IMU_GRAVITY_SIGN * R(q)^T * [0, 0, 1].
static void predict_gravity_stage_4(const double q[4], double h[3])
{
    double w = q[0], x = q[1], y = q[2], z = q[3];

    h[0] = IMU_GRAVITY_SIGN * 2.0 * (x * z - w * y);
    h[1] = IMU_GRAVITY_SIGN * 2.0 * (y * z + w * x);
    h[2] = IMU_GRAVITY_SIGN * (w * w - x * x - y * y + z * z);
}

// EKF STAGE 5 - Measurement: accelerometer normalized to a unit vector
static void normalize_accel_stage_5(const double a[3], double z_meas[3])
{
    double magnitude = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);

    for (int i = 0; i < 3; i++) {
        z_meas[i] = (magnitude < 1e-9) ? 0.0 : a[i] / magnitude;
    }
}

// EKF STAGE 6 - Residual (innovation): y = z - h(q)
static void residual_stage_6(const double z_meas[3], const double h[3], double residual[3])
{
    for (int i = 0; i < 3; i++) {
        residual[i] = z_meas[i] - h[i];
    }
}

// EKF STAGE 7 - Measurement Jacobian H = d h(q) / d [w, x, y, z] (matches stage 4)
static void measurement_jacobian_h_stage_7(const double q[4], double H[3][4])
{
    const double s = IMU_GRAVITY_SIGN;
    double w = q[0], x = q[1], y = q[2], z = q[3];

    double jacobian[3][4] = {
        {s * -2.0 * y, s *  2.0 * z, s * -2.0 * w, s * 2.0 * x},
        {s *  2.0 * x, s *  2.0 * w, s *  2.0 * z, s * 2.0 * y},
        {s *  2.0 * w, s * -2.0 * x, s * -2.0 * y, s * 2.0 * z}
    };

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            H[i][j] = jacobian[i][j];
        }
    }
}

// EKF STAGE 8 - Innovation covariance: S = H P H^T + R
static void innovation_covariance_stage_8(double H[3][4], double P[4][4], double R[3][3], double S[3][3])
{
    double HP[3][4];
    double H_transpose[4][3];

    mat_transpose(3, 4, H, H_transpose);
    mat_mul(3, 4, 4, H, P, HP);
    mat_mul(3, 4, 3, HP, H_transpose, S);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            S[i][j] += R[i][j];
        }
    }
}

// EKF STAGE 9 - Kalman gain: K = P H^T S^-1
// Returns false (K untouched) if S is singular; the caller then skips the correction.
static bool kalman_gain_stage_9(double P[4][4], double H[3][4], double S[3][3], double K[4][3])
{
    double determinant =
          S[0][0] * (S[1][1] * S[2][2] - S[1][2] * S[2][1])
        - S[0][1] * (S[1][0] * S[2][2] - S[1][2] * S[2][0])
        + S[0][2] * (S[1][0] * S[2][1] - S[1][1] * S[2][0]);

    if (fabs(determinant) < 1e-12) {
        return false;
    }

    // 3x3 inverse via the adjugate
    double S_inverse[3][3] = {
        {(S[1][1] * S[2][2] - S[1][2] * S[2][1]) / determinant,
         (S[0][2] * S[2][1] - S[0][1] * S[2][2]) / determinant,
         (S[0][1] * S[1][2] - S[0][2] * S[1][1]) / determinant},
        {(S[1][2] * S[2][0] - S[1][0] * S[2][2]) / determinant,
         (S[0][0] * S[2][2] - S[0][2] * S[2][0]) / determinant,
         (S[0][2] * S[1][0] - S[0][0] * S[1][2]) / determinant},
        {(S[1][0] * S[2][1] - S[1][1] * S[2][0]) / determinant,
         (S[0][1] * S[2][0] - S[0][0] * S[2][1]) / determinant,
         (S[0][0] * S[1][1] - S[0][1] * S[1][0]) / determinant}
    };

    double H_transpose[4][3];
    double P_H_transpose[4][3];

    mat_transpose(3, 4, H, H_transpose);
    mat_mul(4, 4, 3, P, H_transpose, P_H_transpose);
    mat_mul(4, 3, 3, P_H_transpose, S_inverse, K);

    return true;
}

// EKF STAGE 10 - Quaternion correction: q <- normalize(q + K y)
static void correct_quaternion_stage_10(double q[4], double K[4][3], const double residual[3])
{
    for (int i = 0; i < 4; i++) {
        q[i] += K[i][0] * residual[0] + K[i][1] * residual[1] + K[i][2] * residual[2];
    }

    normalize_quaternion(q);
}

// EKF STAGE 11 - Covariance update, Joseph form:
//   P <- (I - K H) P (I - K H)^T + K R K^T
// Mathematically equal to (I - K H) P, but keeps P symmetric and positive
// definite despite rounding. A final symmetrization removes the remaining drift.
static void update_covariance_stage_11(double P[4][4], double K[4][3], double H[3][4], double R[3][3])
{
    double KH[4][4];
    double A[4][4];             // I - K H
    double A_transpose[4][4];
    double AP[4][4];
    double APAt[4][4];
    double KR[4][3];
    double K_transpose[3][4];
    double KRKt[4][4];

    mat_mul(4, 3, 4, K, H, KH);

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            A[i][j] = (i == j ? 1.0 : 0.0) - KH[i][j];
        }
    }

    mat_transpose(4, 4, A, A_transpose);
    mat_mul(4, 4, 4, A, P, AP);
    mat_mul(4, 4, 4, AP, A_transpose, APAt);

    mat_transpose(4, 3, K, K_transpose);
    mat_mul(4, 3, 3, K, R, KR);
    mat_mul(4, 3, 4, KR, K_transpose, KRKt);

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P[i][j] = APAt[i][j] + KRKt[i][j];
        }
    }

    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 4; j++) {
            double average = 0.5 * (P[i][j] + P[j][i]);
            P[i][j] = average;
            P[j][i] = average;
        }
    }
}

// =====================================================================
// EKF - PUBLIC API
// =====================================================================
void ekf_init(ekf_t *ekf)
{
    ekf->q[0] = 1.0;
    ekf->q[1] = 0.0;
    ekf->q[2] = 0.0;
    ekf->q[3] = 0.0;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ekf->P[i][j] = (i == j) ? EKF_P0 : 0.0;
            ekf->Q[i][j] = (i == j) ? EKF_Q : 0.0;
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ekf->R[i][j] = (i == j) ? EKF_R : 0.0;
        }
    }
}

bool ekf_update(ekf_t *ekf, const double gyro[3], const double accel[3], double dt)
{
    double F[4][4];
    double H[3][4];
    double S[3][3];
    double K[4][3];

    double h[3];
    double z_meas[3];
    double residual[3];

    // Prediction (gyro)
    predict_quaternion_stage_1(ekf->q, gyro, dt);
    state_jacobian_f_stage_2(gyro, dt, F);
    predict_covariance_stage_3(ekf->P, F, ekf->Q);

    // Accelerometer gate: only a gravity reference when |a| is close to 1 g
    double accel_magnitude = sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

    if (fabs(accel_magnitude - 1.0) > ACCEL_GATE_G) {
        return false;
    }

    // Correction (accelerometer)
    predict_gravity_stage_4(ekf->q, h);
    normalize_accel_stage_5(accel, z_meas);
    residual_stage_6(z_meas, h, residual);
    measurement_jacobian_h_stage_7(ekf->q, H);
    innovation_covariance_stage_8(H, ekf->P, ekf->R, S);

    if (!kalman_gain_stage_9(ekf->P, H, S, K)) {
        return false;
    }

    correct_quaternion_stage_10(ekf->q, K, residual);
    update_covariance_stage_11(ekf->P, K, H, ekf->R);

    return true;
}

double ekf_radar_angle_deg(const ekf_t *ekf)
{
    const double *q = ekf->q;
    double cos_tilt = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

    cos_tilt = fmax(-1.0, fmin(1.0, cos_tilt));

    return acos(cos_tilt) * RAD_TO_DEG;
}
