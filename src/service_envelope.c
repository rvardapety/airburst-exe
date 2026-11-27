/*
 * service_envelope.c - detect an object in 0.10-0.40 m (TARGET_M +/- TOLERANCE_M) with the A111 Envelope service.
 *
 * - RSS is activated through the existing airburst_init()/airburst_destroy().
 * - The Envelope service measures reflected amplitude versus distance.
 * - At startup an empty-scene background (mean and noise per distance sample) is
 *   recorded. A frame counts as "object present" when the envelope exceeds
 *   background + max(THRESHOLD_K * noise, THRESHOLD_MIN) somewhere inside
 *   TARGET_M +/- TOLERANCE_M.
 * - Hysteresis (DETECT_MS / CLEAR_MS, converted to frames) makes "Object detected at X m"
 *   print once per arrival, not once per frame. X is the envelope peak of the
 *   reflection, refined between samples with a parabolic fit; the peak itself must
 *   lie inside the region (a tail from an object outside it does not count).
 * - Every frame is also sent as one non-blocking UDP packet to the PC viewer
 *   (tools/envelope_view.py). If no viewer listens the packets are dropped;
 *   acquisition never waits for the UI.
 *
 * Usage: envelope_exe [viewer_ip] [viewer_port] [log.csv] [run_s] [rate_hz] [range_end_m] [hwaas] [downsampling]
 *        rate_hz <= 0 selects on-demand mode (as fast as the host loop runs); range_end_m > 7.0 selects MUR 9.
 */

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "acc_service.h"
#include "acc_service_envelope.h"
#include "acc_libgpiod.h"
#include "airburst.h"

/* ---------------- Adjustable settings ---------------- */

/* Radar */
#define SENSOR_ID          1                     /* XC112 port the XR112 is connected to */
#define RANGE_START_M      0.10f                 /* measured window start; below ~0.10 m direct TX leakage saturates the receiver */
#define RANGE_END_DEFAULT_M 0.55f                /* requested window end (0.10-0.55 m); override with argv[6] */
#define RANGE_LENGTH_MAX_M  7.7f                 /* Envelope service length limit (User Guide 3.4.1) - not enforced here */
#define MUR6_MMD_M          7.0f                 /* MUR 6 reaches 7.0 m; beyond that MUR 9 (12.7 m) is selected */
#define PROFILE            ACC_SERVICE_PROFILE_1 /* shortest pulse: shortest direct-leakage tail, best for close range */
#define HWAAS_DEFAULT      10                    /* hardware averaging per sample (1-63); argv[7] */
#define DOWNSAMPLING_DEFAULT 1                   /* 1, 2 or 4: sample step 0.48 / 0.97 / 1.94 mm; argv[8] */
#define RECEIVER_GAIN      0.25f                 /* lowered from 0.5: a hand at ~0.15 m saturated the receiver */
#define UPDATE_RATE_DEFAULT_HZ 100.0f            /* streaming rate requested from the sensor; argv[5]; <= 0 = on demand, flat out */
#define RUNNING_AVG_FACTOR 0.0f                  /* 0 = raw frames, closer to 1 = smoother but slower */

/* Detection */
#define TARGET_M           0.25f /* centre of the detection region */
#define TOLERANCE_M        0.15f /* region = TARGET_M +/- TOLERANCE_M = 0.10-0.40 m */
/*
 * Timing is set in milliseconds and converted to frame counts after calibration, from the
 * measured frame period, so behaviour is the same at any update rate. At 100 Hz the counts
 * are the previous fixed values: 200 / 2 / 5 frames, window 6, 4 points, hold 3, 5 cm jump.
 */
#define CALIBRATION_MS     2000.0 /* empty-scene time used for the background */
#define CALIBRATION_MIN_FRAMES 20 /* at least this many frames, even at low rates */
#define THRESHOLD_K        6.0f  /* threshold = background + max(K * noise, MIN) */
#define THRESHOLD_MIN      30.0f /* minimum margin above background, envelope units */
#define DETECT_MS          20.0  /* above threshold this long before "detected" */
#define CLEAR_MS           50.0  /* below threshold this long before "cleared" */

/* Trigger output */
#define LED_PIN            16    /* BCM16 = physical pin 36; configured as output (PIN_BURST) by the airburst board HAL */
#define LED_MIN_ON_MS      100.0 /* keep the LED on at least this long so a camera cannot miss it */

/* Approaching / receding (radial speed from the distance track) */
#define VEL_WINDOW_MS         60.0  /* time span of the line fit */
#define VEL_WINDOW_MAX        256   /* array size cap for the fit (2.5 ms frames at 60 ms) */
#define VEL_THRESHOLD_MPS     0.05f /* slower than this = not moving */
#define TRACK_MAX_SPEED_MPS   5.0f  /* larger jump per frame period = different object, restart track */
#define DIRECTION_HOLD_MS     30.0  /* a new direction must hold this long before it is reported */

typedef enum
{
	DIR_UNKNOWN   = 0,
	DIR_APPROACH  = 1,
	DIR_RECEDE    = 2,
	DIR_NOT_MOVING = 3,
} direction_t;

static const char *direction_name(direction_t d)
{
	switch (d)
	{
		case DIR_APPROACH:
			return "approaching";
		case DIR_RECEDE:
			return "receding";
		case DIR_NOT_MOVING:
			return "not moving";
		default:
			return "direction unknown";
	}
}

/* Viewer link */
#define VIEWER_DEFAULT_IP   "192.168.0.123"
#define VIEWER_DEFAULT_PORT 5005
#define UI_DECIMATION       2 /* send every Nth sample to the viewer (detection uses all) */

/* ---------------- End of settings ---------------- */

#define PACKET_MAGIC        0x56453141u /* "A1EV" */
#define PACKET_VERSION      3u
#define PACKET_HEADER_BYTES 92u

#define LOG_DEFAULT_PATH "envelope_log.csv"
#define LOG_CAPACITY     200000u /* frames kept in RAM for the CSV log (33 min at 100 Hz) */

/* Flags in frame_record_t.flags */
#define REC_MISSED     0x01u
#define REC_SATURATED  0x02u
#define REC_ABOVE      0x04u
#define REC_DETECTED   0x08u
#define REC_EVENT      0x10u
#define REC_CALIBRATED 0x20u

/* One row of the per-frame log, kept in RAM during the run (no file I/O while measuring). */
typedef struct
{
	uint32_t frame;
	float    t_ms;        /* get_next returned, ms since start (CLOCK_MONOTONIC) */
	float    get_next_ms; /* time blocked in acc_service_envelope_get_next */
	float    proc_ms;     /* detection + packet build, until just before sendto */
	float    band_over;   /* max(envelope - threshold) inside the region (negative = below) */
	float    gpeak_m;     /* strongest above-threshold reflection anywhere in the window, -1 = none */
	float    peak_m;      /* reflection peak inside the region used for detection, -1 = none */
	float    led_ms;      /* detection frames only: get_next return -> LED switched on, else 0 */
	float    velocity;    /* radial speed of the tracked reflection, m/s (negative = approaching), 0 = unknown */
	uint8_t  direction;   /* direction_t after the hold filter */
	uint16_t gpeak_amp;
	uint8_t  flags;
} frame_record_t;

static volatile sig_atomic_t stop_requested = 0;


static void handle_stop_signal(int signal_number)
{
	(void)signal_number;
	stop_requested = 1;
}


static double now_s(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}


static uint8_t *put_bytes(uint8_t *p, const void *value, size_t size)
{
	memcpy(p, value, size);
	return p + size;
}


static uint16_t clamp_u16(float value)
{
	if (value < 0.0f)
	{
		return 0;
	}

	if (value > 65535.0f)
	{
		return 65535;
	}

	return (uint16_t)(value + 0.5f);
}


/*
 * Distance of the envelope peak at sample i, refined between samples with a
 * 3-point parabolic fit (sub-sample accuracy). Falls back to the sample itself
 * at the window edges or when the three points do not form a peak.
 */
static float peak_distance_m(const uint16_t *env, int i, int n, float start_m, float step_m)
{
	float offset = 0.0f;

	if ((i > 0) && (i < n - 1))
	{
		const float a     = (float)env[i - 1];
		const float b     = (float)env[i];
		const float c     = (float)env[i + 1];
		const float denom = a - 2.0f * b + c;

		if (denom < 0.0f)
		{
			offset = 0.5f * (a - c) / denom;
		}
	}

	return start_m + ((float)i + offset) * step_m;
}


/* From sample i, follow the envelope uphill to its local peak; returns the centre of a flat top. */
static int climb_to_peak(const uint16_t *env, int n, int i)
{
	for (;;)
	{
		if ((i + 1 < n) && (env[i + 1] > env[i]))
		{
			i++;
		}
		else if ((i > 0) && (env[i - 1] > env[i]))
		{
			i--;
		}
		else
		{
			break;
		}
	}

	int left  = i;
	int right = i;
	while ((left > 0) && (env[left - 1] == env[i]))
	{
		left--;
	}

	while ((right + 1 < n) && (env[right + 1] == env[i]))
	{
		right++;
	}

	return (left + right) / 2;
}


/*
 * Least-squares slope (m/s) of the last `count` track points, stored in a ring
 * buffer of `size` entries whose next write position is `head`.
 */
static float track_slope(const double *t, const float *d, int size, int head, int count)
{
	double st = 0.0, sd = 0.0, stt = 0.0, std_ = 0.0;

	for (int k = 0; k < count; k++)
	{
		const int    i  = (head - 1 - k + 2 * size) % size;
		const double tk = t[i] - t[(head - 1 + size) % size]; /* relative time keeps precision */
		st   += tk;
		sd   += (double)d[i];
		stt  += tk * tk;
		std_ += tk * (double)d[i];
	}

	const double denom = (double)count * stt - st * st;

	return (denom > 0.0) ? (float)(((double)count * std_ - st * sd) / denom) : 0.0f;
}


static int compare_float(const void *a, const void *b)
{
	const float x = *(const float *)a;
	const float y = *(const float *)b;

	return (x > y) - (x < y);
}


/* Prints min / mean / p50 / p99 / max of a column; values is reordered. */
static void print_stats(const char *name, float *values, size_t count)
{
	if (count == 0)
	{
		return;
	}

	double sum = 0.0;
	for (size_t i = 0; i < count; i++)
	{
		sum += (double)values[i];
	}

	qsort(values, count, sizeof(float), compare_float);
	printf("  %-22s min %8.3f  mean %8.3f  p50 %8.3f  p99 %8.3f  max %8.3f ms\n", name, (double)values[0],
	       sum / (double)count, (double)values[count / 2], (double)values[(count * 99) / 100],
	       (double)values[count - 1]);
}


static void write_log_and_summary(const frame_record_t *rec, size_t count, size_t dropped, const char *path)
{
	FILE *f = fopen(path, "w");

	if (f != NULL)
	{
		fprintf(f, "frame,t_ms,get_next_ms,proc_ms,missed,saturated,calibrated,above,detected,event,"
		           "band_over,peak_m,gpeak_m,gpeak_amp,led_ms,velocity_mps,direction\n");
		for (size_t i = 0; i < count; i++)
		{
			const frame_record_t *r = &rec[i];
			fprintf(f, "%u,%.3f,%.3f,%.3f,%u,%u,%u,%u,%u,%u,%.1f,%.4f,%.4f,%u,%.4f,%.3f,%s\n", (unsigned)r->frame,
			        (double)r->t_ms, (double)r->get_next_ms, (double)r->proc_ms, (r->flags & REC_MISSED) ? 1u : 0u,
			        (r->flags & REC_SATURATED) ? 1u : 0u, (r->flags & REC_CALIBRATED) ? 1u : 0u,
			        (r->flags & REC_ABOVE) ? 1u : 0u, (r->flags & REC_DETECTED) ? 1u : 0u,
			        (r->flags & REC_EVENT) ? 1u : 0u, (double)r->band_over, (double)r->peak_m, (double)r->gpeak_m,
			        (unsigned)r->gpeak_amp, (double)r->led_ms, (double)r->velocity,
			        direction_name((direction_t)r->direction));
		}

		fclose(f);
		printf("Log: %zu frames written to %s%s\n", count, path, dropped ? " (log full, later frames not logged)" : "");
	}
	else
	{
		perror("cannot write log");
	}

	if (count < 3)
	{
		return;
	}

	float *tmp = malloc(count * sizeof(float));
	if (tmp == NULL)
	{
		return;
	}

	printf("Timing on the Pi (%zu frames):\n", count);
	for (size_t i = 1; i < count; i++)
	{
		tmp[i - 1] = rec[i].t_ms - rec[i - 1].t_ms;
	}

	print_stats("frame interval", tmp, count - 1);
	for (size_t i = 0; i < count; i++)
	{
		tmp[i] = rec[i].get_next_ms;
	}

	print_stats("get_next blocked", tmp, count);
	for (size_t i = 0; i < count; i++)
	{
		tmp[i] = rec[i].proc_ms;
	}

	print_stats("processing", tmp, count);

	size_t n_led = 0;
	for (size_t i = 0; i < count; i++)
	{
		if ((rec[i].flags & REC_EVENT) != 0)
		{
			tmp[n_led++] = rec[i].led_ms;
		}
	}

	print_stats("frame -> LED on", tmp, n_led);
	free(tmp);
}


static int open_viewer_socket(const char *ip, int port, struct sockaddr_in *address)
{
	int fd = socket(AF_INET, SOCK_DGRAM, 0);

	if (fd < 0)
	{
		perror("viewer socket");
		return -1;
	}

	int flags = fcntl(fd, F_GETFL, 0);
	if ((flags < 0) || (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0))
	{
		perror("viewer socket non-blocking");
		close(fd);
		return -1;
	}

	memset(address, 0, sizeof(*address));
	address->sin_family = AF_INET;
	address->sin_port   = htons((uint16_t)port);
	if (inet_pton(AF_INET, ip, &address->sin_addr) != 1)
	{
		fprintf(stderr, "Invalid viewer IP address: %s\n", ip);
		close(fd);
		return -1;
	}

	return fd;
}


/* Number of frames that covers time_ms at the given frame period, clamped to [lo, hi]. */
static int frames_for(double time_ms, double period_ms, int lo, int hi)
{
	int frames = (int)lround(time_ms / period_ms);

	if (frames < lo)
	{
		frames = lo;
	}

	if (frames > hi)
	{
		frames = hi;
	}

	return frames;
}


int main(int argc, char *argv[])
{
	const char *viewer_ip   = (argc > 1) ? argv[1] : VIEWER_DEFAULT_IP;
	int         viewer_port = (argc > 2) ? atoi(argv[2]) : VIEWER_DEFAULT_PORT;
	const char *log_path    = (argc > 3) ? argv[3] : LOG_DEFAULT_PATH;
	const double run_s      = (argc > 4) ? atof(argv[4]) : 0.0; /* 0 = run until Ctrl+C */
	const float  rate_hz    = (argc > 5) ? (float)atof(argv[5]) : UPDATE_RATE_DEFAULT_HZ;
	const float  range_end  = (argc > 6) ? (float)atof(argv[6]) : RANGE_END_DEFAULT_M;
	const float  length_req = range_end - RANGE_START_M;
	const bool   streaming  = rate_hz > 0.0f;
	const bool   use_mur9   = range_end > MUR6_MMD_M;
	const int    hwaas      = (argc > 7) ? atoi(argv[7]) : HWAAS_DEFAULT;
	const int    downsample = (argc > 8) ? atoi(argv[8]) : DOWNSAMPLING_DEFAULT;

	/* Frame counts derived from the *_MS settings after calibration (see frames_for()). */
	int frames_to_detect = 2, frames_to_clear = 5, vel_window = 6, vel_min_points = 4, dir_hold_frames = 3;
	float track_max_jump_m = 0.05f;

	frame_record_t *records       = calloc(LOG_CAPACITY, sizeof(frame_record_t));
	size_t          record_count  = 0;
	size_t          records_extra = 0;

	int                         exit_code      = EXIT_FAILURE;
	bool                        rss_active     = false;
	bool                        service_active = false;
	acc_service_configuration_t config         = NULL;
	acc_service_handle_t        handle         = NULL;
	uint16_t                   *envelope       = NULL;
	float                      *bg_mean        = NULL;
	float                      *bg_m2          = NULL;
	float                      *threshold      = NULL;
	uint8_t                    *packet         = NULL;
	int                         viewer_fd      = -1;
	struct sockaddr_in          viewer_address;

	signal(SIGINT, handle_stop_signal);
	signal(SIGTERM, handle_stop_signal);

	if (!airburst_init())
	{
		goto cleanup;
	}

	rss_active = true;

	config = acc_service_envelope_configuration_create();
	if (config == NULL)
	{
		fprintf(stderr, "Envelope configuration creation failed\n");
		goto cleanup;
	}

	acc_service_sensor_set(config, SENSOR_ID);
	acc_service_requested_start_set(config, RANGE_START_M);
	acc_service_requested_length_set(config, length_req);
	acc_service_profile_set(config, PROFILE);
	acc_service_hw_accelerated_average_samples_set(config, (uint8_t)hwaas);
	acc_service_envelope_downsampling_factor_set(config, (uint16_t)downsample);
	acc_service_receiver_gain_set(config, RECEIVER_GAIN);
	acc_service_mur_set(config, use_mur9 ? ACC_SERVICE_MUR_9 : ACC_SERVICE_MUR_6);
	if (streaming)
	{
		acc_service_repetition_mode_streaming_set(config, rate_hz);
	}
	else
	{
		acc_service_repetition_mode_on_demand_set(config);
	}
	acc_service_envelope_running_average_factor_set(config, RUNNING_AVG_FACTOR);

	printf("Requested: %.3f-%.3f m (length %.3f m%s), MUR %d, %s %.0f Hz\n", (double)RANGE_START_M,
	       (double)range_end, (double)length_req, (length_req > RANGE_LENGTH_MAX_M + 0.001f) ? ", above the 7.7 m limit" : "",
	       use_mur9 ? 9 : 6, streaming ? "streaming" : "on demand", (double)rate_hz);

	handle = acc_service_create(config);
	acc_service_envelope_configuration_destroy(&config);
	if (handle == NULL)
	{
		fprintf(stderr, "Envelope service creation failed (sensor on port %d? range/rate accepted by RSS?)\n", SENSOR_ID);
		goto cleanup;
	}

	acc_service_envelope_metadata_t metadata;
	acc_service_envelope_get_metadata(handle, &metadata);

	const uint16_t n    = metadata.data_length;
	const float    step = metadata.step_length_m;

	/* Detection region as sample indices. */
	int region_lo = (int)lroundf((TARGET_M - TOLERANCE_M - metadata.start_m) / step);
	int region_hi = (int)lroundf((TARGET_M + TOLERANCE_M - metadata.start_m) / step);
	const bool detect_enabled = (region_lo >= 0) && (region_hi < (int)n) && (region_lo < region_hi);
	if (!detect_enabled)
	{
		/* Short windows (rate tests): measure and time frames only, no detection. */
		fprintf(stderr, "Detection region %.2f-%.2f m is outside the measured window: measuring timing only\n",
		        (double)(TARGET_M - TOLERANCE_M), (double)(TARGET_M + TOLERANCE_M));
		region_lo = 0;
		region_hi = (int)n - 1;
	}

	envelope  = calloc(n, sizeof(*envelope));
	bg_mean   = calloc(n, sizeof(*bg_mean));
	bg_m2     = calloc(n, sizeof(*bg_m2));
	threshold = calloc(n, sizeof(*threshold));

	const uint16_t n_ui         = (uint16_t)((n + UI_DECIMATION - 1) / UI_DECIMATION);
	const size_t   packet_bytes = PACKET_HEADER_BYTES + 3u * 2u * n_ui;
	packet                      = calloc(packet_bytes, 1);

	if ((envelope == NULL) || (bg_mean == NULL) || (bg_m2 == NULL) || (threshold == NULL) || (packet == NULL))
	{
		fprintf(stderr, "Out of memory\n");
		goto cleanup;
	}

	viewer_fd = open_viewer_socket(viewer_ip, viewer_port, &viewer_address);
	if (viewer_fd < 0)
	{
		fprintf(stderr, "Continuing without viewer\n");
	}

	printf("Envelope: %.3f-%.3f m, %u samples, step %.2f mm, profile %d, HWAAS %d, downsampling %d, gain %.2f, %.0f Hz\n",
	       (double)metadata.start_m, (double)(metadata.start_m + metadata.length_m), (unsigned)n,
	       (double)(step * 1000.0f), (int)PROFILE, hwaas, downsample, (double)RECEIVER_GAIN, (double)rate_hz);
	printf("Detection region: %.2f-%.2f m; viewer: %s:%d\n", (double)(TARGET_M - TOLERANCE_M),
	       (double)(TARGET_M + TOLERANCE_M), viewer_ip, viewer_port);

	if (!acc_service_activate(handle))
	{
		fprintf(stderr, "Envelope service activation failed\n");
		goto cleanup;
	}

	service_active = true;

	printf("Calibrating background for %.1f s: keep the area in front of the sensor clear...\n",
	       CALIBRATION_MS / 1000.0);
	fflush(stdout);

	const double t_start          = now_s();
	double       t_previous       = t_start;
	double       last_detection_t = -1.0;
	float        fps              = 0.0f;
	uint32_t     frame            = 0;
	uint32_t     calibration_n    = 0;
	bool         calibrated       = false;
	bool         detected         = false;
	int          above_count      = 0;
	int          below_count      = 0;
	uint32_t     detection_count  = 0;
	uint32_t     missed_count     = 0;
	uint32_t     saturated_count  = 0;
	double       t_measure_start  = 0.0;
	bool         led_on           = false;
	double       t_led_on         = 0.0;

	/* distance track of the strongest reflection, for the radial speed */
	double      trk_t[VEL_WINDOW_MAX];
	float       trk_d[VEL_WINDOW_MAX];
	int         trk_n         = 0;
	int         trk_head      = 0;
	direction_t dir_stable    = DIR_UNKNOWN;
	direction_t dir_candidate = DIR_UNKNOWN;
	int         dir_hold      = 0;
	direction_t dir_reported  = DIR_UNKNOWN;

	while (!stop_requested)
	{
		if ((run_s > 0.0) && calibrated && ((now_s() - t_measure_start) >= run_s))
		{
			break;
		}

		acc_service_envelope_result_info_t info;
		const double                       t_call = now_s();

		if (!acc_service_envelope_get_next(handle, envelope, n, &info))
		{
			fprintf(stderr, "acc_service_envelope_get_next failed\n");
			goto cleanup;
		}

		if (info.sensor_communication_error)
		{
			fprintf(stderr, "Sensor communication error\n");
			goto cleanup;
		}

		missed_count    += info.missed_data ? 1u : 0u;
		saturated_count += info.data_saturated ? 1u : 0u;

		const double t_now = now_s();
		const double dt    = t_now - t_previous;
		t_previous = t_now;
		if (dt > 0.0)
		{
			fps = (fps == 0.0f) ? (float)(1.0 / dt) : 0.9f * fps + 0.1f * (float)(1.0 / dt);
		}

		frame++;

		float    peak_m      = -1.0f;
		float    peak_excess = 0.0f;
		float    band_over   = -1.0e9f;
		float    gpeak_m     = -1.0f;
		uint16_t gpeak_amp   = 0;
		bool     above       = false;
		bool     event       = false;
		float    led_ms_frame = 0.0f;
		float    velocity     = 0.0f;

		if (!calibrated)
		{
			/* Welford running mean and variance per sample. */
			calibration_n++;
			for (uint16_t i = 0; i < n; i++)
			{
				const float x     = (float)envelope[i];
				const float delta = x - bg_mean[i];
				bg_mean[i] += delta / (float)calibration_n;
				bg_m2[i]   += delta * (x - bg_mean[i]);
			}

			if ((calibration_n >= CALIBRATION_MIN_FRAMES) && ((t_now - t_start) * 1000.0 >= CALIBRATION_MS))
			{
				/* Convert the millisecond settings to frames using the measured frame period. */
				const double period_ms = (t_now - t_start) * 1000.0 / (double)calibration_n;
				frames_to_detect = frames_for(DETECT_MS, period_ms, 1, 1000000);
				frames_to_clear  = frames_for(CLEAR_MS, period_ms, 1, 1000000);
				dir_hold_frames  = frames_for(DIRECTION_HOLD_MS, period_ms, 1, 1000000);
				vel_window       = frames_for(VEL_WINDOW_MS, period_ms, 4, VEL_WINDOW_MAX);
				vel_min_points   = (int)lround(vel_window * 2.0 / 3.0);
				if (vel_min_points < 3)
				{
					vel_min_points = 3;
				}
				track_max_jump_m = TRACK_MAX_SPEED_MPS * (float)(period_ms / 1000.0);
				printf("Frame period %.3f ms (%.1f Hz): detect %d, clear %d frames; velocity fit %d frames (min %d), "
				       "hold %d, max jump %.1f mm\n",
				       period_ms, 1000.0 / period_ms, frames_to_detect, frames_to_clear, vel_window, vel_min_points,
				       dir_hold_frames, (double)(track_max_jump_m * 1000.0f));

				for (uint16_t i = 0; i < n; i++)
				{
					const float sigma  = sqrtf(bg_m2[i] / (float)(calibration_n - 1));
					const float margin = fmaxf(THRESHOLD_K * sigma, THRESHOLD_MIN);
					threshold[i] = bg_mean[i] + margin;
				}

				calibrated     = true;
				t_measure_start = t_now;
				if (detect_enabled)
				{
					printf("Calibration done. Watching %.2f-%.2f m.\n", (double)(TARGET_M - TOLERANCE_M),
					       (double)(TARGET_M + TOLERANCE_M));
				}
				else
				{
					printf("Calibration done. Timing only (no detection).\n");
				}
				if (run_s > 0.0)
				{
					printf("Measuring for %.1f s...\n", run_s);
				}
				fflush(stdout);
			}
		}
		else if (detect_enabled)
		{
			/* Highest envelope sample that is above its threshold inside the detection region. */
			uint16_t best   = 0;
			int      best_i = -1;
			for (int i = region_lo; i <= region_hi; i++)
			{
				if (((float)envelope[i] > threshold[i]) && (envelope[i] > best))
				{
					best   = envelope[i];
					best_i = i;
				}
			}

			/*
			 * Follow the envelope uphill (over the whole window) to the real peak of that
			 * reflection, so the tail of an object just outside the region is not counted
			 * and the reported distance is not clipped to the region edge.
			 */
			const int peak_i = (best_i >= 0) ? climb_to_peak(envelope, (int)n, best_i) : -1;

			/* A "peak" on the first or last sample is a slope running out of the window
			 * (e.g. the direct-leakage tail), not a real reflection peak: ignore it. */
			above = (best_i >= 0) && (peak_i > 0) && (peak_i < (int)n - 1) && (peak_i >= region_lo) &&
			        (peak_i <= region_hi) && ((float)envelope[peak_i] > threshold[peak_i]);

			/* Diagnostics only (do not affect the decision): band margin and strongest reflection anywhere. */
			float g_best = 0.0f;
			int   g_i    = -1;
			for (int i = 0; i < (int)n; i++)
			{
				const float over = (float)envelope[i] - threshold[i];
				if ((i >= region_lo) && (i <= region_hi) && (over > band_over))
				{
					band_over = over;
				}

				if (over > g_best)
				{
					g_best = over;
					g_i    = i;
				}
			}

			if (g_i >= 0)
			{
				const int g_peak = climb_to_peak(envelope, (int)n, g_i);
				gpeak_m   = peak_distance_m(envelope, g_peak, (int)n, metadata.start_m, step);
				gpeak_amp = envelope[g_peak];
			}

			/* Track the strongest reflection's distance and fit its radial speed. */
			if (gpeak_m > 0.0f)
			{
				const int last = (trk_head + vel_window - 1) % vel_window;
				if ((trk_n > 0) && (fabsf(gpeak_m - trk_d[last]) > track_max_jump_m))
				{
					trk_n = 0; /* jumped to a different reflection */
				}

				trk_t[trk_head] = t_now - t_start;
				trk_d[trk_head] = gpeak_m;
				trk_head        = (trk_head + 1) % vel_window;
				if (trk_n < vel_window)
				{
					trk_n++;
				}
			}
			else
			{
				trk_n = 0;
			}

			direction_t dir_raw = DIR_UNKNOWN;
			if (trk_n >= vel_min_points)
			{
				velocity = track_slope(trk_t, trk_d, vel_window, trk_head, trk_n);
				dir_raw  = (velocity < -VEL_THRESHOLD_MPS) ? DIR_APPROACH :
				           (velocity > VEL_THRESHOLD_MPS)  ? DIR_RECEDE :
				                                             DIR_NOT_MOVING;
			}

			if (dir_raw == dir_candidate)
			{
				dir_hold++;
			}
			else
			{
				dir_candidate = dir_raw;
				dir_hold      = 1;
			}

			if ((dir_raw == DIR_UNKNOWN) || (dir_hold >= dir_hold_frames))
			{
				dir_stable = dir_candidate;
			}

			if (above)
			{
				peak_m      = peak_distance_m(envelope, peak_i, (int)n, metadata.start_m, step);
				peak_excess = (float)envelope[peak_i] - bg_mean[peak_i];
				above_count++;
				below_count = 0;
			}
			else
			{
				below_count++;
				above_count = 0;
			}

			if (!detected && (above_count >= frames_to_detect))
			{
				/* Trigger first, print after: the terminal (ssh over Wi-Fi) must not delay the LED. */
				acc_libgpiod_set(LED_PIN, PIN_HIGH);
				led_on       = true;
				t_led_on     = now_s();
				led_ms_frame = (float)((t_led_on - t_now) * 1000.0);

				detected = true;
				event    = true;
				detection_count++;
				last_detection_t = t_now - t_start;

				/* Direction at the moment of detection: the filtered one if known yet, else the latest fit. */
				dir_reported = (dir_stable != DIR_UNKNOWN) ? dir_stable : dir_raw;
				if (dir_reported == DIR_UNKNOWN)
				{
					printf("Object detected at %.3f m - direction unknown\n", (double)peak_m);
				}
				else
				{
					printf("Object detected at %.3f m - %s (%.2f m/s)\n", (double)peak_m, direction_name(dir_reported),
					       (double)fabsf(velocity));
				}

				fflush(stdout);
			}
			else if (detected && (below_count >= frames_to_clear))
			{
				detected     = false;
				dir_reported = DIR_UNKNOWN;
			}
			else if (detected && (dir_stable != DIR_UNKNOWN) && (dir_stable != dir_reported))
			{
				/* Still in the zone but the direction changed: report it once. */
				dir_reported = dir_stable;
				printf("Object at %.3f m - %s (%.2f m/s)\n", (double)((peak_m > 0.0f) ? peak_m : gpeak_m),
				       direction_name(dir_stable), (double)fabsf(velocity));
				fflush(stdout);
			}
		}

		/* LED off once the object is gone and the minimum on-time has passed. */
		if (led_on && !detected && ((now_s() - t_led_on) * 1000.0 >= LED_MIN_ON_MS))
		{
			acc_libgpiod_set(LED_PIN, PIN_LOW);
			led_on = false;
		}

		/* One non-blocking UDP packet per frame for the viewer. */
		if (viewer_fd >= 0)
		{
			const uint32_t magic   = PACKET_MAGIC;
			const uint16_t version = PACKET_VERSION;
			const float    step_ui = step * (float)UI_DECIMATION;
			const float    lo_m    = TARGET_M - TOLERANCE_M;
			const float    hi_m    = TARGET_M + TOLERANCE_M;
			const uint8_t  flags[4] = {detected ? 1 : 0, calibrated ? 1 : 0, info.data_saturated ? 1 : 0,
			                           info.missed_data ? 1 : 0};
			const double   t_rel   = t_now - t_start;
			uint8_t       *p       = packet;

			p = put_bytes(p, &magic, 4);
			p = put_bytes(p, &version, 2);
			p = put_bytes(p, &n_ui, 2);
			p = put_bytes(p, &frame, 4);
			p = put_bytes(p, &metadata.start_m, 4);
			p = put_bytes(p, &step_ui, 4);
			p = put_bytes(p, &lo_m, 4);
			p = put_bytes(p, &hi_m, 4);
			p = put_bytes(p, &fps, 4);
			p = put_bytes(p, &peak_m, 4);
			p = put_bytes(p, &peak_excess, 4);
			p = put_bytes(p, flags, 4);
			p = put_bytes(p, &detection_count, 4);
			p = put_bytes(p, &t_rel, 8);
			p = put_bytes(p, &last_detection_t, 8);

			const float   get_next_ms = (float)((t_now - t_call) * 1000.0);
			const float   proc_ms     = (float)((now_s() - t_now) * 1000.0);
			const float   gamp        = (float)gpeak_amp;
			const uint8_t extra[4]    = {above ? 1 : 0, event ? 1 : 0, (uint8_t)dir_stable, 0};
			p = put_bytes(p, &get_next_ms, 4);
			p = put_bytes(p, &proc_ms, 4);
			p = put_bytes(p, &band_over, 4);
			p = put_bytes(p, &gpeak_m, 4);
			p = put_bytes(p, &gamp, 4);
			p = put_bytes(p, extra, 4);
			p = put_bytes(p, &velocity, 4);

			for (uint16_t k = 0; k < n_ui; k++)
			{
				const uint16_t v = envelope[k * UI_DECIMATION];
				p = put_bytes(p, &v, 2);
			}

			for (uint16_t k = 0; k < n_ui; k++)
			{
				const uint16_t v = calibrated ? clamp_u16(bg_mean[k * UI_DECIMATION]) : 0;
				p = put_bytes(p, &v, 2);
			}

			for (uint16_t k = 0; k < n_ui; k++)
			{
				const uint16_t v = calibrated ? clamp_u16(threshold[k * UI_DECIMATION]) : 0;
				p = put_bytes(p, &v, 2);
			}

			if ((sendto(viewer_fd, packet, (size_t)(p - packet), 0, (const struct sockaddr *)&viewer_address,
			            sizeof(viewer_address)) < 0) &&
			    (errno != EAGAIN) && (errno != EWOULDBLOCK) && (errno != ECONNREFUSED))
			{
				/* Viewer problems never stop acquisition. */
			}
		}

		/* Per-frame record (RAM only; written to CSV at exit). */
		if ((records != NULL) && (record_count < LOG_CAPACITY))
		{
			frame_record_t *r = &records[record_count++];

			r->frame       = frame;
			r->t_ms        = (float)((t_now - t_start) * 1000.0);
			r->get_next_ms = (float)((t_now - t_call) * 1000.0);
			r->proc_ms     = (float)((now_s() - t_now) * 1000.0);
			r->band_over   = calibrated ? band_over : 0.0f;
			r->gpeak_m     = gpeak_m;
			r->peak_m      = peak_m;
			r->gpeak_amp   = gpeak_amp;
			r->led_ms      = event ? led_ms_frame : 0.0f;
			r->velocity    = velocity;
			r->direction   = (uint8_t)dir_stable;
			r->flags       = (uint8_t)((info.missed_data ? REC_MISSED : 0u) | (info.data_saturated ? REC_SATURATED : 0u) |
			                           (calibrated ? REC_CALIBRATED : 0u) | (above ? REC_ABOVE : 0u) |
			                           (detected ? REC_DETECTED : 0u) | (event ? REC_EVENT : 0u));
		}
		else
		{
			records_extra++;
		}
	}

	printf("\nStopped after %u frames: %u detections, %u missed, %u saturated, %.1f Hz\n", (unsigned)frame,
	       (unsigned)detection_count, (unsigned)missed_count, (unsigned)saturated_count, (double)fps);
	exit_code = EXIT_SUCCESS;

cleanup:
	if (service_active)
	{
		acc_service_deactivate(handle);
	}

	if (handle != NULL)
	{
		acc_service_destroy(&handle);
	}

	if (config != NULL)
	{
		acc_service_envelope_configuration_destroy(&config);
	}

	if (viewer_fd >= 0)
	{
		close(viewer_fd);
	}

	if (rss_active)
	{
		acc_libgpiod_set(LED_PIN, PIN_LOW); /* never leave the trigger output on */
	}

	if ((records != NULL) && (record_count > 0))
	{
		write_log_and_summary(records, record_count, records_extra, log_path);
	}

	free(records);
	free(envelope);
	free(bg_mean);
	free(bg_m2);
	free(threshold);
	free(packet);

	if (rss_active)
	{
		airburst_destroy();
	}

	return exit_code;
}
