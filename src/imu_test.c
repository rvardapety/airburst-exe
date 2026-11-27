#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "airburst.h"
// #include "esp32_cam.h"
#include "functions.h"

// Timing: IMU + EKF run every IMU_PERIOD_NS (100 Hz). The state machine, radar
// and printout run every CONTROL_DIVIDER-th sample (5 Hz), so the stability
// window stays STABLE_MEASUREMENT_COUNT * 0.2 s = 2 s.
#define IMU_PERIOD_NS   10000000L
#define CONTROL_DIVIDER 20
#define NS_PER_S        1000000000L

static const double DISTANCE_THRESHOLD_M = 1.0;

static volatile sig_atomic_t running = 1;

static double q_w = 1.0;
static double q_x = 0.0;
static double q_y = 0.0;
static double q_z = 0.0;

static double gyro_bias_x = 0.0;
static double gyro_bias_y = 0.0;
static double gyro_bias_z = 0.0;

static double P[4][4] = {
    {0.01, 0.0, 0.0, 0.0},
    {0.0, 0.01, 0.0, 0.0},
    {0.0, 0.0, 0.01, 0.0},
    {0.0, 0.0, 0.0, 0.01}
};

static double Q[4][4] = {
    {0.0001, 0.0, 0.0, 0.0},
    {0.0, 0.0001, 0.0, 0.0},
    {0.0, 0.0, 0.0001, 0.0},
    {0.0, 0.0, 0.0, 0.0001}
};

static double R[3][3] = {{0.0}};
static double H[3][4] = {{0.0}};
static double F[4][4] = {{0.0}};
static double S[3][3] = {{0.0}};
static double K[4][3] = {{0.0}};

static void handle_stop_signal(int signal_number)
{
    (void)signal_number;
    running = 0;
}

static void install_signal_handlers(void)
{
    struct sigaction action;

    action.sa_handler = handle_stop_signal;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0; // no SA_RESTART: interrupt sleeps so Ctrl+C is handled at once

    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);
}

static void timespec_add_ns(struct timespec *time, long ns)
{
    time->tv_nsec += ns;

    while (time->tv_nsec >= NS_PER_S) {
        time->tv_nsec -= NS_PER_S;
        time->tv_sec++;
    }
}

// Seconds from 'earlier' to 'later' (negative if 'later' is before 'earlier')
static double timespec_diff_s(const struct timespec *later, const struct timespec *earlier)
{
    return (double)(later->tv_sec - earlier->tv_sec) + (double)(later->tv_nsec - earlier->tv_nsec) / 1e9;
}

static const char *state_name(airburst_state_t state)
{
    switch (state) {
    case UNSTABLE:  return "UNSTABLE";
    case STABLE:    return "STABLE";
    case MEASURING: return "MEASURING";
    default:        return "UNKNOWN";
    }
}

// Main simulation loop. Returns true if the burst LED fired, false if a stop signal arrived.
static bool run_simulation(void)
{
    gyro_value_t gyro = {0.0f, 0.0f, 0.0f};
    accel_value_t accel = {0.0f, 0.0f, 0.0f};

    struct timespec previous_time;
    struct timespec current_time;
    struct timespec next_wakeup;

    int sample_count = 0;
    bool fired = false;

    clock_gettime(CLOCK_MONOTONIC, &previous_time);
    next_wakeup = previous_time;

    while (running) {
        timespec_add_ns(&next_wakeup, IMU_PERIOD_NS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup, NULL);

        // Fell behind (e.g. a blocking radar read): restart the schedule from now
        // instead of running several samples back to back to catch up
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (timespec_diff_s(&now, &next_wakeup) > (double)IMU_PERIOD_NS / 1e9) {
            next_wakeup = now;
        }

        // Skip this sample on a failed read; previous_time is not advanced,
        // so the next dt covers the skipped interval
        if (!imu_get_gyro_data(&gyro) || !imu_get_accel_data(&accel)) {
            printf("WARNING: IMU read failed, skipping sample\n");
            continue;
        }

        clock_gettime(CLOCK_MONOTONIC, &current_time);
        double dt = calculate_dt(&previous_time, &current_time);

        // Bias/offset-corrected sensor values, used by both the EKF and the state machine
        double g_x = (double)gyro.gyro_x - gyro_bias_x;
        double g_y = (double)gyro.gyro_y - gyro_bias_y;
        double g_z = (double)gyro.gyro_z - gyro_bias_z;

        double a_x = (double)accel.accel_x - ACCEL_OFFSET_X_G;
        double a_y = (double)accel.accel_y - ACCEL_OFFSET_Y_G;
        double a_z = (double)accel.accel_z - ACCEL_OFFSET_Z_G;

        bool accel_used = ekf_update(&q_w, &q_x, &q_y, &q_z, P, Q, R, H, F, S, K, g_x, g_y, g_z, a_x, a_y, a_z, dt);

        // Control step (5 Hz): state machine, LEDs, radar, printout
        if (++sample_count < CONTROL_DIVIDER) {
            continue;
        }
        sample_count = 0;

        double radar_angle = calculate_radar_angle(q_w, q_x, q_y, q_z);
        double accel_magnitude = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
        double gyro_magnitude = sqrt(g_x * g_x + g_y * g_y + g_z * g_z);

        airburst_state_t state = airburst_fall_state(accel_magnitude, gyro_magnitude, radar_angle);

        printf("Radar angle: %6.2f deg | |a| = %.3f g | |w| = %.3f rad/s | dt = %.4f s%s | STATE: %s",
               radar_angle, accel_magnitude, gyro_magnitude, dt,
               accel_used ? "" : " (gyro only)", state_name(state));

        switch (state) {
        case UNSTABLE:
            airburst_destabilized();
            printf("\n");
            break;

        case STABLE:
            airburst_stabilized();
            printf("\n");
            break;

        case MEASURING: {
            float raw_distance = airburst_get_distance_m();

            // Sensor error or no target: never trigger on an invalid reading
            if (raw_distance < 0.0f) {   // AIRBURST_NO_DISTANCE
                printf(" | Distance: no target\n");
                break;
            }

            double distance = (double)raw_distance;

            printf(" | Distance: %.2f m\n", distance);

            if (distance > 0.0 && distance <= DISTANCE_THRESHOLD_M) {
                airburst_led_on();
                printf("LED ON!\n");
                fired = true;
                running = 0;
            }
            break;
        }

        default:
            printf("\n");
            break;
        }
    }

    return fired;
}

int main(void)
{
    int exit_code = EXIT_FAILURE;
    bool rss_active = false;
    bool distance_active = false;

    install_signal_handlers();

    if (!imu_i2c_init()) {
        printf("ERROR: imu_i2c_init() failed\n");
    } else if (!airburst_init()) {
        printf("ERROR: airburst_init() failed\n");
    } else {
        rss_active = true;

        // esp32_cam_init();

        if (!airburst_distance_init()) {
            // On failure, airburst_distance_init() already deactivates RSS itself
            printf("ERROR: airburst_distance_init() failed\n");
            rss_active = false;
        } else {
            distance_active = true;

            if (!calculate_gyro_bias(&gyro_bias_x, &gyro_bias_y, &gyro_bias_z)) {
                printf("ERROR: gyro bias calibration failed (no IMU data)\n");
            } else {
                printf("Gyro bias: X=%.6f Y=%.6f Z=%.6f rad/s\n", gyro_bias_x, gyro_bias_y, gyro_bias_z);

                if (!run_simulation()) {
                    printf("Stopped by signal\n");
                }
                exit_code = EXIT_SUCCESS;
            }
        }
    }

    // Single cleanup path: leave the LEDs off and release everything that was set up
    if (rss_active) {
        airburst_destabilized();
    }
    if (distance_active) {
        airburst_distance_destroy();
    }
    if (rss_active) {
        airburst_destroy();
    }
    imu_i2c_deinit();

    return exit_code;
}
