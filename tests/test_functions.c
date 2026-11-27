// Unit tests for src/functions.c (EKF, state machine, gyro calibration, dt).
// No hardware needed: imu_get_gyro_data() is replaced by a stub below.
//
// Run on the development PC:  tests/run_host_tests.sh
// Run on the Raspberry Pi:    meson test -C buildDir

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <stdio.h>
#include <time.h>

#include "functions.h"

#define PI 3.14159265358979323846

static int failures = 0;

#define CHECK(condition, ...)                                   \
    do {                                                        \
        if (condition) {                                        \
            printf("  ok    ");                                 \
        } else {                                                \
            printf("  FAIL  ");                                 \
            failures++;                                         \
        }                                                       \
        printf(__VA_ARGS__);                                    \
        printf("\n");                                           \
    } while (0)

// ---------------------------------------------------------------------
// IMU stub for calculate_gyro_bias()
// ---------------------------------------------------------------------
static int stub_mode = 0;   // 0: still (small noise), 1: moving, 2: read fails
static int stub_calls = 0;

bool imu_get_gyro_data(gyro_value_t *gyro)
{
    stub_calls++;

    if (stub_mode == 2) {
        return false;
    }

    float noise = (stub_calls % 2 == 0) ? 0.001f : -0.001f;   // std 0.001 rad/s
    float swing = (stub_mode == 1) ? 0.5f * (float)sin(stub_calls * 0.3) : 0.0f;

    gyro->gyro_x = 0.010f + noise + swing;
    gyro->gyro_y = -0.020f + noise;
    gyro->gyro_z = 0.005f + noise;
    return true;
}

// ---------------------------------------------------------------------
// Helpers: true orientation -> what the IMU would measure
// ---------------------------------------------------------------------

// Rotation about the body X axis by 'tilt', after a yaw about Z; q rotates body -> world
static void make_quaternion(double yaw, double tilt, double q[4])
{
    double cy = cos(yaw / 2), sy = sin(yaw / 2), ct = cos(tilt / 2), st = sin(tilt / 2);

    q[0] = cy * ct;
    q[1] = cy * st;
    q[2] = sy * st;
    q[3] = sy * ct;
}

// Accelerometer reading at rest (g) for orientation q: IMU_GRAVITY_SIGN * R(q)^T [0, 0, 1]
static void accel_at_rest(const double q[4], double a[3])
{
    double w = q[0], x = q[1], y = q[2], z = q[3];

    a[0] = IMU_GRAVITY_SIGN * 2.0 * (x * z - w * y);
    a[1] = IMU_GRAVITY_SIGN * 2.0 * (y * z + w * x);
    a[2] = IMU_GRAVITY_SIGN * (w * w - x * x - y * y + z * z);
}

static void settle(ekf_t *ekf, const double a[3], int steps)
{
    const double no_rotation[3] = {0.0, 0.0, 0.0};

    for (int i = 0; i < steps; i++) {
        ekf_update(ekf, no_rotation, a, 0.01);
    }
}

// ---------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------
static void test_ekf_static_tilt(void)
{
    printf("EKF: converges to a static tilt, independent of yaw\n");

    for (int yaw_deg = 0; yaw_deg <= 90; yaw_deg += 45) {
        ekf_t ekf;
        double q_true[4], a[3];

        ekf_init(&ekf);
        make_quaternion(yaw_deg * PI / 180.0, 30.0 * PI / 180.0, q_true);
        accel_at_rest(q_true, a);
        settle(&ekf, a, 2000);

        double angle = ekf_radar_angle_deg(&ekf);
        CHECK(fabs(angle - 30.0) < 0.5, "yaw %2d deg: angle %.2f deg (expected 30.00)", yaw_deg, angle);
    }

    ekf_t level;
    double a_level[3] = {0.0, 0.0, IMU_GRAVITY_SIGN};

    ekf_init(&level);
    settle(&level, a_level, 500);
    CHECK(ekf_radar_angle_deg(&level) < 0.5, "level: angle %.2f deg (expected 0.00)", ekf_radar_angle_deg(&level));
}

static void test_ekf_accel_gate(void)
{
    printf("EKF: accelerometer gate ignores shakes and free fall\n");

    const double no_rotation[3] = {0.0, 0.0, 0.0};
    double a_level[3] = {0.0, 0.0, IMU_GRAVITY_SIGN};
    double shake[3] = {0.5, 0.0, IMU_GRAVITY_SIGN};
    double free_fall[3] = {0.0, 0.0, 0.0};

    ekf_t ekf;
    ekf_init(&ekf);
    settle(&ekf, a_level, 500);

    bool used = true;
    for (int i = 0; i < 100; i++) {
        used = ekf_update(&ekf, no_rotation, shake, 0.01) && used;
    }
    CHECK(!used, "0.5 g shake: accelerometer correction skipped");
    CHECK(ekf_radar_angle_deg(&ekf) < 0.5, "0.5 g shake for 1 s: angle %.2f deg", ekf_radar_angle_deg(&ekf));

    for (int i = 0; i < 100; i++) {
        ekf_update(&ekf, no_rotation, free_fall, 0.01);
    }
    CHECK(ekf_radar_angle_deg(&ekf) < 0.5, "free fall for 1 s: angle %.2f deg", ekf_radar_angle_deg(&ekf));
}

static void test_ekf_tracking(void)
{
    printf("EKF: tracks a 20 deg / 1 Hz wobble at 100 Hz, with yaw\n");

    for (int yaw_deg = 0; yaw_deg <= 90; yaw_deg += 90) {
        ekf_t ekf;
        double max_error = 0.0;
        double amplitude = 20.0 * PI / 180.0;
        double yaw = yaw_deg * PI / 180.0;
        double q_true[4], a[3];

        ekf_init(&ekf);
        make_quaternion(yaw, 0.0, q_true);
        ekf.q[0] = q_true[0];
        ekf.q[3] = q_true[3];

        for (int i = 1; i <= 500; i++) {
            double t = i * 0.01;
            double tilt = amplitude * sin(2.0 * PI * t);
            double rate = amplitude * 2.0 * PI * cos(2.0 * PI * t);
            double g[3] = {rate, 0.0, 0.0};   // body X rotation rate

            make_quaternion(yaw, tilt, q_true);
            accel_at_rest(q_true, a);
            ekf_update(&ekf, g, a, 0.01);

            double error = fabs(ekf_radar_angle_deg(&ekf) - fabs(tilt) * 180.0 / PI);
            max_error = fmax(max_error, error);
        }

        CHECK(max_error < 1.5, "yaw %2d deg: max tracking error %.2f deg", yaw_deg, max_error);
    }
}

static void test_ekf_covariance(void)
{
    printf("EKF: covariance stays symmetric and positive\n");

    ekf_t ekf;
    double q_true[4], a[3];

    ekf_init(&ekf);

    for (int i = 1; i <= 20000; i++) {
        double t = i * 0.01;
        double tilt = 0.3 * sin(t);
        double g[3] = {0.3 * cos(t), 0.0, 0.0};

        make_quaternion(0.0, tilt, q_true);
        accel_at_rest(q_true, a);
        ekf_update(&ekf, g, a, 0.01);
    }

    double max_asymmetry = 0.0;
    double min_diagonal = ekf.P[0][0];

    for (int i = 0; i < 4; i++) {
        min_diagonal = fmin(min_diagonal, ekf.P[i][i]);
        for (int j = 0; j < 4; j++) {
            max_asymmetry = fmax(max_asymmetry, fabs(ekf.P[i][j] - ekf.P[j][i]));
        }
    }

    CHECK(max_asymmetry == 0.0, "after 20000 updates: max |P - P^T| = %g", max_asymmetry);
    CHECK(min_diagonal > 0.0, "after 20000 updates: min diagonal of P = %g", min_diagonal);
}

static void test_state_machine(void)
{
    printf("State machine: transitions, no counter wrap, reset on disturbance\n");

    int stable_at = 0, measuring_at = 0, unstable_after = 0;

    for (int step = 1; step <= 600; step++) {
        double accel = (step == 300) ? 1.5 : 1.0;   // one disturbance at step 300
        airburst_state_t state = airburst_fall_state(accel, 0.0, 0.0);

        if (state == STABLE && stable_at == 0) {
            stable_at = step;
        }
        if (state == MEASURING && measuring_at == 0) {
            measuring_at = step;
        }
        if (step > STABLE_MEASUREMENT_COUNT + 1 && step < 300 && state == UNSTABLE) {
            unstable_after = step;   // would indicate the old uint8_t counter wrap
        }
        if (step == 300) {
            CHECK(state == UNSTABLE, "disturbance at step 300 -> UNSTABLE");
        }
        if (step == 310) {
            CHECK(state == STABLE, "10 steps after disturbance -> STABLE again");
        }
        if (step == 600) {
            CHECK(state == MEASURING, "step 600 -> MEASURING");
        }
    }

    CHECK(stable_at == STABLE_MEASUREMENT_COUNT, "STABLE first at step %d (expected %d)", stable_at, STABLE_MEASUREMENT_COUNT);
    CHECK(measuring_at == STABLE_MEASUREMENT_COUNT + 1, "MEASURING first at step %d (expected %d)", measuring_at, STABLE_MEASUREMENT_COUNT + 1);
    CHECK(unstable_after == 0, "no drop to UNSTABLE between steps 12 and 299 (old counter wrap)");

    airburst_state_t tilted = airburst_fall_state(1.0, 0.0, RADAR_ANGLE_THRESHOLD_DEG + 1.0);
    CHECK(tilted == UNSTABLE, "tilt above threshold -> UNSTABLE");
}

static void test_gyro_bias(void)
{
    printf("Gyro calibration: bias, motion detection, read failure\n");

    double bias[3];
    double std_rad;

    stub_mode = 0;
    stub_calls = 0;
    bool ok = calculate_gyro_bias(bias, &std_rad);
    CHECK(ok, "still: returns true");
    CHECK(fabs(bias[0] - 0.010) < 1e-6 && fabs(bias[1] + 0.020) < 1e-6 && fabs(bias[2] - 0.005) < 1e-6,
          "still: bias = (%.4f, %.4f, %.4f), expected (0.0100, -0.0200, 0.0050)", bias[0], bias[1], bias[2]);
    CHECK(std_rad < GYRO_CALIBRATION_MAX_STD_RAD, "still: noise %.4f rad/s below limit", std_rad);

    stub_mode = 1;
    calculate_gyro_bias(bias, &std_rad);
    CHECK(std_rad > GYRO_CALIBRATION_MAX_STD_RAD, "moving: noise %.4f rad/s above limit", std_rad);

    stub_mode = 2;
    CHECK(!calculate_gyro_bias(bias, &std_rad), "read failure: returns false");
}

static void test_dt(void)
{
    printf("dt calculation\n");

    struct timespec previous = {.tv_sec = 10, .tv_nsec = 990000000L};
    struct timespec current = {.tv_sec = 11, .tv_nsec = 5000000L};

    double dt = calculate_dt(&previous, &current);
    CHECK(fabs(dt - 0.015) < 1e-9, "dt across a second boundary = %.4f s (expected 0.0150)", dt);
    CHECK(previous.tv_sec == 11 && previous.tv_nsec == 5000000L, "previous_time advanced to current_time");
}

int main(void)
{
    test_ekf_static_tilt();
    test_ekf_accel_gate();
    test_ekf_tracking();
    test_ekf_covariance();
    test_state_machine();
    test_gyro_bias();
    test_dt();

    printf("\n%s: %d failure(s)\n", failures == 0 ? "PASSED" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
