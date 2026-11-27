#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "airburst.h"
// #include "esp32_cam.h"
#include "functions.h"

// Usage: airburst_exe [--csv]
//   default : one human-readable status line per control step on stdout
//   --csv   : one CSV row per control step on stdout (header first), e.g.
//             ./airburst_exe --csv > run.csv
// Status, warning and error messages always go to stderr.

// Timing: IMU + EKF run every IMU_PERIOD_NS (100 Hz). The state machine, radar
// and output run every CONTROL_DIVIDER-th sample (5 Hz), so the stability
// window stays STABLE_MEASUREMENT_COUNT * 0.2 s = 2 s.
#define IMU_PERIOD_NS   10000000L
#define CONTROL_DIVIDER 20
#define NS_PER_S        1000000000L

static const double DISTANCE_THRESHOLD_M = 1.0;

static volatile sig_atomic_t running = 1;

static ekf_t ekf;
static double gyro_bias[3] = {0.0, 0.0, 0.0};
static bool csv_output = false;

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

// One output line per control step.
// distance_m: NAN = not measured in this step, AIRBURST_NO_DISTANCE (< 0) = no target
static void print_status(double time_s, double dt, double radar_angle, double accel_magnitude,
                         double gyro_magnitude, bool accel_used, airburst_state_t state, double distance_m)
{
    if (csv_output) {
        printf("%.3f,%.4f,%.2f,%.3f,%.4f,%d,%s,", time_s, dt, radar_angle, accel_magnitude,
               gyro_magnitude, accel_used ? 1 : 0, state_name(state));
        if (!isnan(distance_m)) {
            printf("%.3f", distance_m);
        }
        printf("\n");
        return;
    }

    printf("Radar angle: %6.2f deg | |a| = %.3f g | |w| = %.3f rad/s | dt = %.4f s%s | STATE: %s",
           radar_angle, accel_magnitude, gyro_magnitude, dt, accel_used ? "" : " (gyro only)", state_name(state));

    if (isnan(distance_m)) {
        printf("\n");
    } else if (distance_m < 0.0) {
        printf(" | Distance: no target\n");
    } else {
        printf(" | Distance: %.2f m\n", distance_m);
    }
}

// Main simulation loop. Returns true if the burst LED fired, false if a stop signal arrived.
static bool run_simulation(void)
{
    gyro_value_t gyro = {0.0f, 0.0f, 0.0f};
    accel_value_t accel = {0.0f, 0.0f, 0.0f};

    struct timespec start_time;
    struct timespec previous_time;
    struct timespec current_time;
    struct timespec next_wakeup;

    int sample_count = 0;
    bool fired = false;

    if (csv_output) {
        printf("time_s,dt_s,angle_deg,accel_g,gyro_rad_s,accel_used,state,distance_m\n");
    }

    clock_gettime(CLOCK_MONOTONIC, &start_time);
    previous_time = start_time;
    next_wakeup = start_time;

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
        if (!imu_get_data(&accel, &gyro)) {
            fprintf(stderr, "WARNING: IMU read failed, skipping sample\n");
            continue;
        }

        clock_gettime(CLOCK_MONOTONIC, &current_time);
        double dt = calculate_dt(&previous_time, &current_time);

        // Bias/offset-corrected sensor values, used by both the EKF and the state machine
        double g[3] = {
            (double)gyro.gyro_x - gyro_bias[0],
            (double)gyro.gyro_y - gyro_bias[1],
            (double)gyro.gyro_z - gyro_bias[2]
        };
        double a[3] = {
            (double)accel.accel_x - ACCEL_OFFSET_X_G,
            (double)accel.accel_y - ACCEL_OFFSET_Y_G,
            (double)accel.accel_z - ACCEL_OFFSET_Z_G
        };

        bool accel_used = ekf_update(&ekf, g, a, dt);

        // Control step (5 Hz): state machine, LEDs, radar, output
        if (++sample_count < CONTROL_DIVIDER) {
            continue;
        }
        sample_count = 0;

        double radar_angle = ekf_radar_angle_deg(&ekf);
        double accel_magnitude = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        double gyro_magnitude = sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
        double distance = NAN;

        airburst_state_t state = airburst_fall_state(accel_magnitude, gyro_magnitude, radar_angle);

        switch (state) {
        case UNSTABLE:
            airburst_destabilized();
            break;

        case STABLE:
            airburst_stabilized();
            break;

        case MEASURING:
            // AIRBURST_NO_DISTANCE (< 0) on sensor error or no target: never triggers
            distance = (double)airburst_get_distance_m();
            break;

        default:
            break;
        }

        print_status(timespec_diff_s(&current_time, &start_time), dt, radar_angle, accel_magnitude,
                     gyro_magnitude, accel_used, state, distance);

        if (state == MEASURING && distance > 0.0 && distance <= DISTANCE_THRESHOLD_M) {
            airburst_led_on();
            fprintf(stderr, "LED ON!\n");
            fired = true;
            running = 0;
        }
    }

    return fired;
}

int main(int argc, char **argv)
{
    int exit_code = EXIT_FAILURE;
    bool rss_active = false;
    bool distance_active = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--csv") == 0) {
            csv_output = true;
        } else {
            fprintf(stderr, "Usage: %s [--csv]\n", argv[0]);
            return EXIT_FAILURE;
        }
    }

    install_signal_handlers();
    ekf_init(&ekf);

    if (!imu_i2c_init()) {
        fprintf(stderr, "ERROR: imu_i2c_init() failed\n");
    } else if (!airburst_init()) {
        fprintf(stderr, "ERROR: airburst_init() failed\n");
    } else {
        rss_active = true;

        // esp32_cam_init();

        if (!airburst_distance_init()) {
            // On failure, airburst_distance_init() already deactivates RSS itself
            fprintf(stderr, "ERROR: airburst_distance_init() failed\n");
            rss_active = false;
        } else {
            distance_active = true;

            double gyro_std_rad = 0.0;

            if (!calculate_gyro_bias(gyro_bias, &gyro_std_rad)) {
                fprintf(stderr, "ERROR: gyro bias calibration failed (no IMU data)\n");
            } else {
                fprintf(stderr, "Gyro bias: X=%.6f Y=%.6f Z=%.6f rad/s (noise %.4f rad/s)\n",
                        gyro_bias[0], gyro_bias[1], gyro_bias[2], gyro_std_rad);

                if (gyro_std_rad > GYRO_CALIBRATION_MAX_STD_RAD) {
                    fprintf(stderr, "WARNING: device moved during gyro calibration "
                                    "(%.4f > %.4f rad/s); bias may be wrong, restart with the device still\n",
                            gyro_std_rad, GYRO_CALIBRATION_MAX_STD_RAD);
                }

                if (!run_simulation()) {
                    fprintf(stderr, "Stopped by signal\n");
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
