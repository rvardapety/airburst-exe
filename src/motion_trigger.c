/*
 * motion_trigger.c - motion trigger (hand wave, thrown ball, ...) with distance, LED output and a remote-controlled UI.
 *
 * Based on Acconeer's ref_app_wave_to_exit.c (RSS 2.15.4): the presence detector (Sparse service) reports
 * a motion score every frame; a trigger fires once when the score rises above TRIGGER_THRESHOLD and re-arms
 * when it falls below REARM_THRESHOLD (and the cool-down time has passed). Static objects do not trigger.
 *
 * Added on top of the example:
 *  - distance of the motion: the library's presence_distance (~6 cm grid) and an interpolated distance
 *    from the per-point score vector (parabolic fit around the strongest point);
 *  - LED / trigger output on BCM16 with a minimum on-time;
 *  - optional distance zone: trigger only inside [zone_lo, zone_hi];
 *  - timing: frame interval, missed frames, get_next time, processing time; per-frame CSV log; summary at exit;
 *  - automatic restart after a sensor communication error / failed get_next;
 *  - TCP control (one text command per line) and a UDP frame stream for tools/trigger_view.py.
 *
 * Why two kinds of settings (see the "Wave Trigger: What Can Change While Measuring" doc):
 *  The RSS API copies the configuration into the detector at create/reconfigure and has no setters on a
 *  running handle. Radar settings are therefore staged with "set" and take effect with "apply"
 *  (deactivate -> reconfigure -> activate). Trigger/zone/LED/distance settings are applied by this program
 *  to the score and point vector, so they change instantly with no gap in the data.
 *
 * Usage: trigger_exe [udp_port] [tcp_port] [log.csv] [viewer_ip]
 *   udp_port  frame stream destination port on the viewer PC        (default 5006)
 *   tcp_port  control port this program listens on                 (default 5007)
 *   log.csv   CSV log path; logging starts enabled when given       (default off, path trigger_log.csv)
 *   viewer_ip where to stream before a UI connects                  (default: none; the IP of the
 *             connecting UI is used automatically)
 *
 * Control commands (reply is one line starting with "ok" or "err", followed by the full configuration):
 *   start | stop | restart | apply | revert | get | defaults | set <key> <value>
 *   restart = from the very beginning: LED off, detector destroyed, RSS re-activated, detector re-created
 *             with the current radar settings, counters/statistics/timeline reset, measuring started.
 */

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "acc_detector_presence.h"
#include "acc_libgpiod.h"
#include "airburst.h"

/* ---------------- Defaults (Wave to exit values unless noted) ---------------- */
/* User defaults (2026-11-16): range, rate, sweeps, HWAAS, profile and LED time differ from the example. */

#define SENSOR_ID            1
#define DEF_START_M          0.20f /* user default; below ~0.2 m profile 3 direct leakage saturates */
#define DEF_LENGTH_M         2.92f /* 0.20-3.12 m (Wave to exit: 0.12-0.30 m) */
#define DEF_RATE_HZ          250.0f /* measured reachable with these settings (Wave to exit: 80) */
#define DEF_SWEEPS_PER_FRAME 11   /* (Wave to exit: 32) */
#define DEF_HWAAS            10   /* (Wave to exit: 60) */
#define DEF_PROFILE          3    /* (Wave to exit: 2) */
#define DEF_POWER_SAVE       1 /* 0 off, 1 sleep, 2 ready, 3 active, 4 hibernate */
#define DEF_DOWNSAMPLING     1
#define DEF_INTRA_WEIGHT     0.0f  /* MTI default (2026-11-16): inter-frame part only (Wave to exit: 1.0) */
#define DEF_INTER_DEV_TC     0.02f /* MTI: fast reaction to a passing object (library default 0.5 s) */
#define DEF_INTER_SLOW_HZ    10.0f /* MTI: changes slower than ~10 Hz (still objects, breathing, sitting) removed */
#define DEF_INTER_FAST_HZ    120.0f /* MTI: keep changes up to 120 Hz (must stay < rate / 2) */
#define DEF_INTRA_TC         0.05f
#define DEF_OUTPUT_TC        0.02f

#define DEF_TRIGGER_THR      3.0f /* MTI: idle up to ~1.6, small after-pass bumps up to ~3, passes 10-40 (Wave to exit: 1.4) */
#define DEF_REARM_THR        1.5f /* (Wave to exit: 1.1) */
#define DEF_COOLDOWN_MS      0.0f
#define DEF_REARM_QUIET_MS   300.0f /* measured: 3 passes -> 3 triggers; a held ball 5 instead of 21 */
#define DEF_LED_ON_MS        106.0f /* user default */

#define LED_PIN              16     /* BCM16, physical pin 36 */
#define SPARSE_STEP_M        0.06f  /* sparse base point spacing (header: "~6cm") */
#define MAX_POINTS           512
#define SMOOTH_MAX           32
#define WARMUP_MIN_FRAMES    3
#define RESTART_RETRY_MS     1000.0

#define DEF_UDP_PORT         5006
#define DEF_TCP_PORT         5007
#define LOG_DEFAULT_PATH     "trigger_log.csv"

#define PACKET_MAGIC         0x56573141u /* "A1WV" */
#define PACKET_VERSION       1u

/* ---------------- Configuration ---------------- */

typedef struct
{
	float    start_m;
	float    length_m;
	float    rate_hz;
	uint16_t sweeps_per_frame;
	float    sweep_rate;       /* <= 0 = maximum */
	int      profile;          /* 1..5 */
	int      hwaas;            /* 1..63 */
	float    gain;             /* 0..1 */
	int      downsampling;     /* >= 1 */
	int      power_save;       /* 0..4 */
	float    inter_dev_tc;
	float    inter_fast_cutoff;
	float    inter_slow_cutoff;
	float    intra_tc;
	float    intra_weight;
	float    output_tc;
	int      nbr_removed_pc;   /* 0..2 */
	int      async;            /* 0/1 */
} radar_cfg_t;

typedef struct
{
	float trigger_thr;
	float rearm_thr;
	float cooldown_ms;
	float rearm_quiet_ms; /* score must stay below the re-arm level this long before re-arming */
	int   zone_on;
	float zone_lo;
	float zone_hi;
	int   led_on;
	float led_on_ms;
	int   dist_mode;     /* 0 library distance, 1 interpolated */
	int   smooth;        /* distance average over N frames, 1 = off */
	int   stream_every;  /* send every Nth frame to the viewer */
	int   log_on;
} live_cfg_t;

typedef enum
{
	K_FLOAT,
	K_INT,
	K_U16,
} key_type_t;

typedef struct
{
	const char *name;
	key_type_t  type;
	bool        radar;  /* true: staged until apply */
	size_t      offset;
	double      lo;
	double      hi;
} key_t_;

#define RK(n, t, f, lo, hi) { n, t, true, offsetof(radar_cfg_t, f), lo, hi }
#define LK(n, t, f, lo, hi) { n, t, false, offsetof(live_cfg_t, f), lo, hi }

static const key_t_ keys[] = {
	RK("start", K_FLOAT, start_m, 0.0, 7.0),
	RK("length", K_FLOAT, length_m, 0.01, 6.7), /* library: range end <= 6.726 m */
	RK("rate", K_FLOAT, rate_hz, 0.1, 5000.0),
	RK("spf", K_U16, sweeps_per_frame, 1, 64), /* measured: >64 rejected; points x sweeps must stay <= ~2048 */
	RK("sweep_rate", K_FLOAT, sweep_rate, 0.0, 100000.0),
	RK("profile", K_INT, profile, 1, 5),
	RK("hwaas", K_INT, hwaas, 1, 63),
	RK("gain", K_FLOAT, gain, 0.0, 1.0),
	RK("downsampling", K_INT, downsampling, 1, 64),
	RK("power_save", K_INT, power_save, 0, 4),
	RK("inter_dev_tc", K_FLOAT, inter_dev_tc, 0.0, 100.0),
	RK("inter_fast", K_FLOAT, inter_fast_cutoff, 0.0, 1000.0),
	RK("inter_slow", K_FLOAT, inter_slow_cutoff, 0.0, 1000.0),
	RK("intra_tc", K_FLOAT, intra_tc, 0.0, 100.0),
	RK("intra_weight", K_FLOAT, intra_weight, 0.0, 1.0),
	RK("output_tc", K_FLOAT, output_tc, 0.0, 100.0),
	RK("pca", K_INT, nbr_removed_pc, 0, 2),
	RK("async", K_INT, async, 0, 1),
	LK("threshold", K_FLOAT, trigger_thr, 0.0, 1000.0),
	LK("rearm", K_FLOAT, rearm_thr, 0.0, 1000.0),
	LK("cooldown_ms", K_FLOAT, cooldown_ms, 0.0, 600000.0),
	LK("rearm_ms", K_FLOAT, rearm_quiet_ms, 0.0, 60000.0),
	LK("zone_on", K_INT, zone_on, 0, 1),
	LK("zone_lo", K_FLOAT, zone_lo, 0.0, 15.0),
	LK("zone_hi", K_FLOAT, zone_hi, 0.0, 15.0),
	LK("led", K_INT, led_on, 0, 1),
	LK("led_on_ms", K_FLOAT, led_on_ms, 0.0, 60000.0),
	LK("dist_mode", K_INT, dist_mode, 0, 1),
	LK("smooth", K_INT, smooth, 1, SMOOTH_MAX),
	LK("stream_every", K_INT, stream_every, 1, 1000),
	LK("log", K_INT, log_on, 0, 1),
};

#define NKEYS (sizeof(keys) / sizeof(keys[0]))

/* ---------------- State ---------------- */

static volatile sig_atomic_t stop_requested = 0;

static radar_cfg_t radar_active;       /* what the detector runs with (last good) */
static radar_cfg_t radar_pending;      /* staged by "set", committed by "apply" */
static radar_cfg_t radar_lib_defaults;
static live_cfg_t  live;

static acc_detector_presence_handle_t handle = NULL;
static bool  running = false;      /* user state: Start pressed */
static bool  sensor_ok = false;    /* detector exists and is active when running */
static double next_retry_ms = 0.0;

/* Trigger state */
static bool   armed = true;
static double rearm_time_ms = -1e12;
static double quiet_since_ms = -1.0; /* when the score last went below the re-arm level; < 0 = not quiet */
static double led_off_at_ms = 0.0;
static bool   led_state = false;
static int    warmup_left = 0;

/* Distance smoothing */
static float  dist_hist[SMOOTH_MAX];
static int    dist_hist_n = 0, dist_hist_head = 0;

/* Counters and timing */
static uint32_t frame_no = 0, trigger_count = 0, restart_count = 0, missed_count = 0, apply_count = 0;
static double   last_frame_ms = 0.0, last_apply_ms = 0.0, last_trigger_dist = 0.0;
static double   stat_int_min = 1e9, stat_int_max = 0.0, stat_int_sum = 0.0;
static uint32_t stat_int_n = 0;
static double   stat_gn_max = 0.0, stat_gn_sum = 0.0, stat_proc_max = 0.0, stat_proc_sum = 0.0;
static uint32_t stat_gn_n = 0;
static double   t0_ms = 0.0;

/* I/O */
#define MAX_CLIENTS 4 /* e.g. the UI and a test script at the same time */
typedef struct
{
	int    fd;
	char   buf[1024];
	size_t len;
} client_t;
static client_t clients[MAX_CLIENTS];
static int  reply_fd = -1; /* client whose command is being processed */
static int  udp_fd = -1, tcp_listen_fd = -1;
static struct sockaddr_in viewer_addr;
static bool viewer_set = false;
static int  udp_port = DEF_UDP_PORT;
static FILE *log_fp = NULL;
static const char *log_path = LOG_DEFAULT_PATH;
static char status_msg[160] = "";

/* ---------------- Helpers ---------------- */

static void handle_stop_signal(int s)
{
	(void)s;
	stop_requested = 1;
}

static double now_ms(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return (double)t.tv_sec * 1000.0 + (double)t.tv_nsec / 1e6;
}

static void led_set(bool on)
{
	if (on != led_state)
	{
		acc_libgpiod_set(LED_PIN, on ? PIN_HIGH : PIN_LOW);
		led_state = on;
	}
}

static acc_power_save_mode_t power_save_enum(int v)
{
	switch (v)
	{
		case 0:
			return ACC_POWER_SAVE_MODE_OFF;
		case 2:
			return ACC_POWER_SAVE_MODE_READY;
		case 3:
			return ACC_POWER_SAVE_MODE_ACTIVE;
		case 4:
			return ACC_POWER_SAVE_MODE_HIBERNATE;
		default:
			return ACC_POWER_SAVE_MODE_SLEEP;
	}
}

static void radar_defaults(radar_cfg_t *r)
{
	*r                  = radar_lib_defaults;
	r->start_m          = DEF_START_M;
	r->length_m         = DEF_LENGTH_M;
	r->rate_hz          = DEF_RATE_HZ;
	r->sweeps_per_frame = DEF_SWEEPS_PER_FRAME;
	r->hwaas            = DEF_HWAAS;
	r->profile          = DEF_PROFILE;
	r->power_save       = DEF_POWER_SAVE;
	r->downsampling     = DEF_DOWNSAMPLING;
	r->intra_weight     = DEF_INTRA_WEIGHT;
	r->intra_tc         = DEF_INTRA_TC;
	r->output_tc        = DEF_OUTPUT_TC;
	r->inter_dev_tc      = DEF_INTER_DEV_TC;
	r->inter_slow_cutoff = DEF_INTER_SLOW_HZ;
	r->inter_fast_cutoff = DEF_INTER_FAST_HZ;
}

static void live_defaults(live_cfg_t *l)
{
	l->trigger_thr  = DEF_TRIGGER_THR;
	l->rearm_thr    = DEF_REARM_THR;
	l->cooldown_ms  = DEF_COOLDOWN_MS;
	l->rearm_quiet_ms = DEF_REARM_QUIET_MS;
	l->zone_on      = 0;
	l->zone_lo      = DEF_START_M;
	l->zone_hi      = DEF_START_M + DEF_LENGTH_M;
	l->led_on       = 1;
	l->led_on_ms    = DEF_LED_ON_MS;
	l->dist_mode    = 1;
	l->smooth       = 1;
	l->stream_every = 1;
	l->log_on       = 0;
}

/* Library defaults for everything the example does not set (gain, sweep rate, inter-frame filters, ...). */
static void read_library_defaults(void)
{
	acc_detector_presence_configuration_t c = acc_detector_presence_configuration_create();
	radar_cfg_t                           *r = &radar_lib_defaults;
	memset(r, 0, sizeof(*r));
	if (c == NULL)
	{
		return;
	}
	acc_detector_presence_configuration_filter_parameters_t f = acc_detector_presence_configuration_filter_parameters_get(c);
	r->start_m           = acc_detector_presence_configuration_start_get(c);
	r->length_m          = acc_detector_presence_configuration_length_get(c);
	r->rate_hz           = acc_detector_presence_configuration_update_rate_get(c);
	r->sweeps_per_frame  = acc_detector_presence_configuration_sweeps_per_frame_get(c);
	r->sweep_rate        = acc_detector_presence_configuration_sweep_rate_get(c);
	r->profile           = (int)acc_detector_presence_configuration_service_profile_get(c);
	r->hwaas             = acc_detector_presence_configuration_hw_accelerated_average_samples_get(c);
	r->gain              = acc_detector_presence_configuration_receiver_gain_get(c);
	r->downsampling      = acc_detector_presence_configuration_downsampling_factor_get(c);
	r->power_save        = (int)acc_detector_presence_configuration_power_save_mode_get(c);
	r->inter_dev_tc      = f.inter_frame_deviation_time_const;
	r->inter_fast_cutoff = f.inter_frame_fast_cutoff;
	r->inter_slow_cutoff = f.inter_frame_slow_cutoff;
	r->intra_tc          = f.intra_frame_time_const;
	r->intra_weight      = f.intra_frame_weight;
	r->output_tc         = f.output_time_const;
	r->nbr_removed_pc    = acc_detector_presence_configuration_nbr_removed_pc_get(c);
	r->async             = acc_detector_presence_configuration_asynchronous_measurement_get(c) ? 1 : 0;
	acc_detector_presence_configuration_destroy(&c);
}

static acc_detector_presence_configuration_t build_config(const radar_cfg_t *r)
{
	acc_detector_presence_configuration_t c = acc_detector_presence_configuration_create();
	if (c == NULL)
	{
		return NULL;
	}
	acc_detector_presence_configuration_sensor_set(c, SENSOR_ID);
	acc_detector_presence_configuration_start_set(c, r->start_m);
	acc_detector_presence_configuration_length_set(c, r->length_m);
	acc_detector_presence_configuration_update_rate_set(c, r->rate_hz);
	acc_detector_presence_configuration_sweeps_per_frame_set(c, r->sweeps_per_frame);
	acc_detector_presence_configuration_sweep_rate_set(c, r->sweep_rate);
	acc_detector_presence_configuration_service_profile_set(c, (acc_service_profile_t)r->profile);
	acc_detector_presence_configuration_hw_accelerated_average_samples_set(c, (uint8_t)r->hwaas);
	acc_detector_presence_configuration_receiver_gain_set(c, r->gain);
	acc_detector_presence_configuration_downsampling_factor_set(c, (uint16_t)r->downsampling);
	acc_detector_presence_configuration_power_save_mode_set(c, power_save_enum(r->power_save));
	acc_detector_presence_configuration_nbr_removed_pc_set(c, (uint8_t)r->nbr_removed_pc);
	acc_detector_presence_configuration_asynchronous_measurement_set(c, r->async != 0);
	acc_detector_presence_configuration_vector_output_mode_set(c, true);
	/* The library flag is not used for triggering (our live threshold is); keep it equal for reference. */
	acc_detector_presence_configuration_detection_threshold_set(c, live.trigger_thr);

	acc_detector_presence_configuration_filter_parameters_t f = acc_detector_presence_configuration_filter_parameters_get(c);
	f.inter_frame_deviation_time_const = r->inter_dev_tc;
	f.inter_frame_fast_cutoff          = r->inter_fast_cutoff;
	f.inter_frame_slow_cutoff          = r->inter_slow_cutoff;
	f.intra_frame_time_const           = r->intra_tc;
	f.intra_frame_weight               = r->intra_weight;
	f.output_time_const                = r->output_tc;
	acc_detector_presence_configuration_filter_parameters_set(c, &f);
	return c;
}

/* Frames until the score filters have settled: 3 x the slowest time constant that feeds the score. */
static int warmup_frames(const radar_cfg_t *r)
{
	float tc = fmaxf(r->intra_tc, r->output_tc);
	if (r->intra_weight < 1.0f)
	{
		tc = fmaxf(tc, r->inter_dev_tc);
	}
	int n = (int)ceilf(3.0f * tc * r->rate_hz);
	return n < WARMUP_MIN_FRAMES ? WARMUP_MIN_FRAMES : n;
}

static void reset_processing(void)
{
	armed         = true;
	rearm_time_ms = -1e12;
	quiet_since_ms = -1.0;
	dist_hist_n   = 0;
	dist_hist_head = 0;
	warmup_left   = warmup_frames(&radar_active);
	last_frame_ms = 0.0;
}

static void destroy_detector(void)
{
	if (handle != NULL)
	{
		acc_detector_presence_deactivate(handle);
		acc_detector_presence_destroy(&handle);
	}
	sensor_ok = false;
}

/* Create the detector from radar_active; activate if running. */
static bool create_detector(void)
{
	acc_detector_presence_configuration_t c = build_config(&radar_active);
	if (c == NULL)
	{
		return false;
	}
	handle = acc_detector_presence_create(c);
	acc_detector_presence_configuration_destroy(&c);
	if (handle == NULL)
	{
		return false;
	}
	if (running && !acc_detector_presence_activate(handle))
	{
		acc_detector_presence_destroy(&handle);
		return false;
	}
	sensor_ok = true;
	reset_processing();
	return true;
}

/* Full recovery after a sensor error: detector, then RSS, retried every RESTART_RETRY_MS. */
static void try_restart(const char *why)
{
	double t = now_ms();
	if (t < next_retry_ms)
	{
		return;
	}
	next_retry_ms = t + RESTART_RETRY_MS;
	restart_count++;
	led_set(false);
	printf("Sensor restart #%u (%s)\n", (unsigned)restart_count, why);
	destroy_detector();
	if (create_detector())
	{
		snprintf(status_msg, sizeof(status_msg), "restarted detector after %s", why);
		return;
	}
	airburst_destroy();
	if (airburst_init() && create_detector())
	{
		snprintf(status_msg, sizeof(status_msg), "restarted RSS after %s", why);
		return;
	}
	snprintf(status_msg, sizeof(status_msg), "restart failed after %s, retrying", why);
}

/* ---------------- Commands ---------------- */

static int format_cfg(char *buf, size_t size)
{
	int n = 0;
	for (size_t i = 0; i < NKEYS && n < (int)size; i++)
	{
		const key_t_ *k    = &keys[i];
		const uint8_t *act = k->radar ? (const uint8_t *)&radar_active : (const uint8_t *)&live;
		const uint8_t *pen = k->radar ? (const uint8_t *)&radar_pending : (const uint8_t *)&live;
		double a = 0, p = 0;
		switch (k->type)
		{
			case K_FLOAT:
				a = (double)*(const float *)(const void *)(act + k->offset);
				p = (double)*(const float *)(const void *)(pen + k->offset);
				break;
			case K_INT:
				a = (double)*(const int *)(const void *)(act + k->offset);
				p = (double)*(const int *)(const void *)(pen + k->offset);
				break;
			case K_U16:
				a = (double)*(const uint16_t *)(const void *)(act + k->offset);
				p = (double)*(const uint16_t *)(const void *)(pen + k->offset);
				break;
		}
		/* radar keys: name=active/pending ; live keys: name=value */
		if (k->radar)
		{
			n += snprintf(buf + n, size - (size_t)n, " %s=%g/%g", k->name, a, p);
		}
		else
		{
			n += snprintf(buf + n, size - (size_t)n, " %s=%g", k->name, a);
		}
	}
	if (n < (int)size)
	{
		n += snprintf(buf + n, size - (size_t)n, " running=%d sensor_ok=%d apply_ms=%.1f", running ? 1 : 0,
		              sensor_ok ? 1 : 0, last_apply_ms);
	}
	return n;
}

static void open_log(void)
{
	if (log_fp != NULL)
	{
		return;
	}
	bool exists = access(log_path, F_OK) == 0;
	log_fp      = fopen(log_path, "a");
	if (log_fp == NULL)
	{
		snprintf(status_msg, sizeof(status_msg), "cannot open log %s", log_path);
		live.log_on = 0;
		return;
	}
	setvbuf(log_fp, NULL, _IOFBF, 1 << 16);
	if (!exists)
	{
		fprintf(log_fp, "frame,t_ms,interval_ms,get_next_ms,proc_ms,score,lib_dist_m,interp_dist_m,out_dist_m,"
		                "lib_detected,armed,in_zone,warmup,trigger,led,saturated,comm_error,threshold,rearm,n_points,points...\n");
	}
}

static void close_log(void)
{
	if (log_fp != NULL)
	{
		fclose(log_fp);
		log_fp = NULL;
	}
}

static bool apply_radar(char *err, size_t err_size)
{
	double t = now_ms();
	led_set(false);
	if (handle != NULL && running)
	{
		acc_detector_presence_deactivate(handle);
	}
	acc_detector_presence_configuration_t c = build_config(&radar_pending);
	bool ok = c != NULL && handle != NULL && acc_detector_presence_reconfigure(&handle, c);
	if (c != NULL)
	{
		acc_detector_presence_configuration_destroy(&c);
	}
	if (ok && running && !acc_detector_presence_activate(handle))
	{
		ok = false;
	}
	if (ok)
	{
		radar_active = radar_pending;
		sensor_ok    = true;
		reset_processing();
	}
	else
	{
		/* Rejected: rebuild with the last good settings so the radar keeps working. */
		snprintf(err, err_size, "reconfigure rejected, restored last good settings");
		destroy_detector();
		radar_pending = radar_active;
		if (!create_detector())
		{
			try_restart("rejected apply");
		}
	}
	last_apply_ms = now_ms() - t;
	apply_count++;
	return ok;
}

/* "restart": everything from the beginning, keeping the current (active) settings. */
static void full_restart(void)
{
	led_set(false);
	destroy_detector();
	airburst_destroy();
	frame_no = trigger_count = restart_count = missed_count = apply_count = 0;
	last_trigger_dist = 0.0;
	stat_int_min = 1e9;
	stat_int_max = stat_int_sum = 0.0;
	stat_int_n   = 0;
	stat_gn_max = stat_gn_sum = stat_proc_max = stat_proc_sum = 0.0;
	stat_gn_n   = 0;
	radar_pending = radar_active;
	running       = true;
	t0_ms         = now_ms();
	next_retry_ms = 0.0;
	if (airburst_init() && create_detector())
	{
		snprintf(status_msg, sizeof(status_msg), "full restart done");
	}
	else
	{
		snprintf(status_msg, sizeof(status_msg), "full restart: sensor not available, retrying");
		next_retry_ms = now_ms() + RESTART_RETRY_MS;
	}
}

static void process_command(char *line)
{
	char reply[3072];
	char err[160] = "";
	bool ok       = true;
	char *save    = NULL;
	char *cmd     = strtok_r(line, " \t\r\n", &save);

	if (cmd == NULL)
	{
		return;
	}
	if (strcmp(cmd, "start") == 0)
	{
		if (!running)
		{
			running = true;
			if (handle != NULL && acc_detector_presence_activate(handle))
			{
				sensor_ok = true;
				reset_processing();
			}
			else
			{
				destroy_detector();
				if (!create_detector())
				{
					try_restart("start");
				}
			}
		}
	}
	else if (strcmp(cmd, "stop") == 0)
	{
		if (running)
		{
			running = false;
			if (handle != NULL)
			{
				acc_detector_presence_deactivate(handle);
			}
			led_set(false);
		}
	}
	else if (strcmp(cmd, "apply") == 0)
	{
		ok = apply_radar(err, sizeof(err));
	}
	else if (strcmp(cmd, "restart") == 0)
	{
		full_restart();
		ok = sensor_ok;
		if (!ok)
		{
			snprintf(err, sizeof(err), "sensor not available, retrying every %.0f ms", RESTART_RETRY_MS);
		}
	}
	else if (strcmp(cmd, "revert") == 0)
	{
		radar_pending = radar_active;
	}
	else if (strcmp(cmd, "defaults") == 0)
	{
		radar_defaults(&radar_pending);
		int log_keep = live.log_on;
		live_defaults(&live);
		live.log_on = log_keep;
	}
	else if (strcmp(cmd, "get") == 0)
	{
	}
	else if (strcmp(cmd, "set") == 0)
	{
		char *name = strtok_r(NULL, " \t\r\n", &save);
		char *val  = strtok_r(NULL, " \t\r\n", &save);
		const key_t_ *k = NULL;
		for (size_t i = 0; name != NULL && i < NKEYS; i++)
		{
			if (strcmp(keys[i].name, name) == 0)
			{
				k = &keys[i];
			}
		}
		char  *end = NULL;
		double v   = val != NULL ? strtod(val, &end) : 0.0;
		if (k == NULL || val == NULL || end == val)
		{
			ok = false;
			snprintf(err, sizeof(err), "usage: set <key> <number>");
		}
		else if (v < k->lo || v > k->hi)
		{
			ok = false;
			snprintf(err, sizeof(err), "%s must be %g..%g", k->name, k->lo, k->hi);
		}
		else
		{
			uint8_t *base = k->radar ? (uint8_t *)&radar_pending : (uint8_t *)&live;
			switch (k->type)
			{
				case K_FLOAT:
					*(float *)(void *)(base + k->offset) = (float)v;
					break;
				case K_INT:
					*(int *)(void *)(base + k->offset) = (int)lround(v);
					break;
				case K_U16:
					*(uint16_t *)(void *)(base + k->offset) = (uint16_t)lround(v);
					break;
			}
			if (strcmp(k->name, "log") == 0)
			{
				if (live.log_on)
				{
					open_log();
				}
				else
				{
					close_log();
				}
			}
		}
	}
	else
	{
		ok = false;
		snprintf(err, sizeof(err), "unknown command %s", cmd);
	}

	int n = snprintf(reply, sizeof(reply), "%s %s%s%s |", ok ? "ok" : "err", cmd, err[0] ? ": " : "", err);
	n += format_cfg(reply + n, sizeof(reply) - (size_t)n - 2);
	if (n > (int)sizeof(reply) - 2)
	{
		n = (int)sizeof(reply) - 2;
	}
	reply[n++] = '\n';
	if (reply_fd >= 0)
	{
		(void)send(reply_fd, reply, (size_t)n, MSG_NOSIGNAL); /* a dead client is noticed by recv() */
	}
}

static void poll_control(void)
{
	/* Accept new connections; up to MAX_CLIENTS at once. The newest client's IP gets the UDP stream. */
	struct sockaddr_in peer;
	socklen_t          len = sizeof(peer);
	int                fd  = accept(tcp_listen_fd, (struct sockaddr *)&peer, &len);
	if (fd >= 0)
	{
		int slot = -1;
		for (int i = 0; i < MAX_CLIENTS; i++)
		{
			if (clients[i].fd < 0)
			{
				slot = i;
				break;
			}
		}
		if (slot < 0)
		{
			close(fd); /* full */
		}
		else
		{
			fcntl(fd, F_SETFL, O_NONBLOCK);
			int one = 1;
			setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
			clients[slot].fd     = fd;
			clients[slot].len    = 0;
			viewer_addr          = peer;
			viewer_addr.sin_port = htons((uint16_t)udp_port);
			viewer_set           = true;
			printf("Client connected from %s, streaming to UDP %d\n", inet_ntoa(peer.sin_addr), udp_port);
		}
	}

	for (int i = 0; i < MAX_CLIENTS; i++)
	{
		client_t *c = &clients[i];
		if (c->fd < 0)
		{
			continue;
		}
		ssize_t r = recv(c->fd, c->buf + c->len, sizeof(c->buf) - 1 - c->len, 0);
		if (r == 0 || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
		{
			printf("Client disconnected\n");
			close(c->fd);
			c->fd = -1;
			continue;
		}
		if (r < 0)
		{
			continue;
		}
		c->len += (size_t)r;
		c->buf[c->len] = '\0';
		char *nl;
		while ((nl = strchr(c->buf, '\n')) != NULL)
		{
			*nl      = '\0';
			reply_fd = c->fd;
			process_command(c->buf);
			reply_fd    = -1;
			size_t used = (size_t)(nl + 1 - c->buf);
			memmove(c->buf, nl + 1, c->len - used + 1);
			c->len -= used;
		}
		if (c->len >= sizeof(c->buf) - 1)
		{
			c->len = 0; /* line too long: drop it */
		}
	}
}

/* ---------------- Frame processing ---------------- */

/*
 * Interpolated distance: position of the strongest point (point i is at start + i * step), refined with a
 * parabolic fit through it and its two neighbours. Computed from the point vector itself, because the
 * library reports presence_distance = 0 whenever it does not detect presence.
 */
static float interp_distance(const float *v, uint16_t n, float lib_dist, float start, float step)
{
	if (n == 0)
	{
		return lib_dist;
	}
	uint16_t imax = 0;
	for (uint16_t i = 1; i < n; i++)
	{
		if (v[i] > v[imax])
		{
			imax = i;
		}
	}
	if (imax == 0 || imax + 1 >= n)
	{
		return start + (float)imax * step; /* edge point: no neighbour on one side */
	}
	float a = v[imax - 1], b = v[imax], c = v[imax + 1];
	float den = a - 2.0f * b + c;
	float delta = den < 0.0f ? 0.5f * (a - c) / den : 0.0f;
	if (delta > 0.5f)
	{
		delta = 0.5f;
	}
	if (delta < -0.5f)
	{
		delta = -0.5f;
	}
	return start + ((float)imax + delta) * step;
}

static uint8_t *put(uint8_t *p, const void *v, size_t s)
{
	memcpy(p, v, s);
	return p + s;
}

static void process_frame(const acc_detector_presence_result_t *res, const float *vec, uint16_t n, double t_before,
                          double t_after)
{
	double t_frame     = t_after;
	double interval    = last_frame_ms > 0.0 ? t_frame - last_frame_ms : 0.0;
	double period      = 1000.0 / (double)radar_active.rate_hz;
	last_frame_ms      = t_frame;
	frame_no++;
	if (interval > 0.0)
	{
		if (interval > 1.5 * period)
		{
			missed_count += (uint32_t)(interval / period + 0.5) - 1u;
		}
		stat_int_sum += interval;
		stat_int_n++;
		stat_int_min = interval < stat_int_min ? interval : stat_int_min;
		stat_int_max = interval > stat_int_max ? interval : stat_int_max;
	}
	double gn = t_after - t_before;

	float step   = SPARSE_STEP_M * (float)radar_active.downsampling;
	float idist  = interp_distance(vec, n, res->presence_distance, radar_active.start_m, step);
	float raw    = live.dist_mode ? idist : res->presence_distance;
	int   sm     = live.smooth < 1 ? 1 : (live.smooth > SMOOTH_MAX ? SMOOTH_MAX : live.smooth);
	dist_hist[dist_hist_head] = raw;
	dist_hist_head            = (dist_hist_head + 1) % SMOOTH_MAX;
	if (dist_hist_n < SMOOTH_MAX)
	{
		dist_hist_n++;
	}
	int   cnt = dist_hist_n < sm ? dist_hist_n : sm;
	float sum = 0.0f;
	for (int i = 0; i < cnt; i++)
	{
		sum += dist_hist[(dist_hist_head - 1 - i + SMOOTH_MAX) % SMOOTH_MAX];
	}
	float out_dist = sum / (float)cnt;

	float score   = res->presence_score;
	bool  in_zone = !live.zone_on || (out_dist >= live.zone_lo && out_dist <= live.zone_hi);
	bool  warm    = warmup_left > 0;
	bool  trigger = false;
	if (warm)
	{
		warmup_left--;
	}
	else
	{
		bool cooled = (t_frame - rearm_time_ms) >= (double)live.cooldown_ms;
		if (armed && cooled && score > live.trigger_thr && in_zone)
		{
			trigger = true;
			armed   = false;
		}
		/* Re-arm only after the score has stayed below the re-arm level for rearm_quiet_ms:
		 * a held object or a body near the sensor dips below it briefly and would re-trigger. */
		if (!armed && !trigger)
		{
			if (score < live.rearm_thr)
			{
				if (quiet_since_ms < 0.0)
				{
					quiet_since_ms = t_frame;
				}
				if (t_frame - quiet_since_ms >= (double)live.rearm_quiet_ms)
				{
					armed          = true;
					rearm_time_ms  = t_frame;
					quiet_since_ms = -1.0;
				}
			}
			else
			{
				quiet_since_ms = -1.0;
			}
		}
	}
	if (trigger)
	{
		if (live.led_on)
		{
			led_set(true);
			led_off_at_ms = now_ms() + (double)live.led_on_ms;
		}
		trigger_count++;
		last_trigger_dist = (double)out_dist;
		printf("Trigger at %.3f m (score %.2f, library %.3f m)\n", (double)out_dist, (double)score,
		       (double)res->presence_distance);
	}
	double proc = now_ms() - t_after; /* includes the LED write: frame-to-LED time */

	stat_gn_sum += gn;
	stat_proc_sum += proc;
	stat_gn_n++;
	stat_gn_max   = gn > stat_gn_max ? gn : stat_gn_max;
	stat_proc_max = proc > stat_proc_max ? proc : stat_proc_max;

	if (log_fp != NULL)
	{
		fprintf(log_fp, "%u,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%d,%d,%d,%d,%d,%d,%d,%d,%.3f,%.3f",
		        (unsigned)frame_no, t_frame - t0_ms, interval, gn, proc, (double)score,
		        (double)res->presence_distance, (double)idist, (double)out_dist, res->presence_detected ? 1 : 0,
		        armed ? 1 : 0, in_zone ? 1 : 0, warm ? 1 : 0, trigger ? 1 : 0, led_state ? 1 : 0,
		        res->data_saturated ? 1 : 0, res->sensor_communication_error ? 1 : 0, (double)live.trigger_thr,
		        (double)live.rearm_thr);
		/* Per-point motion values (the bars in the UI): n, then one value per point, start + i * step. */
		fprintf(log_fp, ",%u", (unsigned)n);
		for (uint16_t i = 0; i < n; i++)
		{
			fprintf(log_fp, ",%.3f", (double)vec[i]);
		}
		fputc('\n', log_fp);
	}

	if (!viewer_set || (frame_no % (uint32_t)live.stream_every) != 0u)
	{
		return;
	}
	uint8_t  pkt[96 + 4 * MAX_POINTS];
	uint8_t *p     = pkt;
	uint32_t magic = PACKET_MAGIC;
	uint16_t ver   = PACKET_VERSION, npts = n > MAX_POINTS ? MAX_POINTS : n;
	uint32_t flags = (running ? 1u : 0u) | (res->presence_detected ? 2u : 0u) | (trigger ? 4u : 0u) |
	                 (armed ? 8u : 0u) | (led_state ? 16u : 0u) | (warm ? 32u : 0u) |
	                 (res->data_saturated ? 64u : 0u) | (in_zone ? 128u : 0u) | (live.zone_on ? 256u : 0u) |
	                 (log_fp != NULL ? 512u : 0u);
	float f_t     = (float)((t_frame - t0_ms) / 1000.0);
	float f_int   = (float)interval, f_gn = (float)gn, f_proc = (float)proc;
	float f_start = radar_active.start_m, f_step = step;
	float f_rate  = stat_int_n > 0 && interval > 0.0 ? (float)(1000.0 / interval) : 0.0f;
	float f_apply = (float)last_apply_ms, f_ltd = (float)last_trigger_dist;
	p = put(p, &magic, 4);
	p = put(p, &ver, 2);
	p = put(p, &npts, 2);
	p = put(p, &frame_no, 4);
	p = put(p, &flags, 4);
	p = put(p, &f_t, 4);
	p = put(p, &score, 4);
	p = put(p, &res->presence_distance, 4);
	p = put(p, &idist, 4);
	p = put(p, &out_dist, 4);
	p = put(p, &live.trigger_thr, 4);
	p = put(p, &live.rearm_thr, 4);
	p = put(p, &live.zone_lo, 4);
	p = put(p, &live.zone_hi, 4);
	p = put(p, &f_start, 4);
	p = put(p, &f_step, 4);
	p = put(p, &f_int, 4);
	p = put(p, &f_gn, 4);
	p = put(p, &f_proc, 4);
	p = put(p, &f_rate, 4);
	p = put(p, &f_apply, 4);
	p = put(p, &f_ltd, 4);
	p = put(p, &trigger_count, 4);
	p = put(p, &restart_count, 4);
	p = put(p, &missed_count, 4);  /* header = 96 bytes */
	p = put(p, vec, 4u * npts);
	sendto(udp_fd, pkt, (size_t)(p - pkt), MSG_DONTWAIT, (const struct sockaddr *)&viewer_addr, sizeof(viewer_addr));
}

static void print_summary(void)
{
	printf("\n--- trigger_exe summary ---\n");
	printf("frames %u, triggers %u, missed frames %u, restarts %u, applies %u\n", (unsigned)frame_no,
	       (unsigned)trigger_count, (unsigned)missed_count, (unsigned)restart_count, (unsigned)apply_count);
	if (stat_int_n > 0)
	{
		printf("frame interval ms: min %.2f mean %.2f max %.2f\n", stat_int_min, stat_int_sum / stat_int_n,
		       stat_int_max);
	}
	if (stat_gn_n > 0)
	{
		printf("get_next ms: mean %.2f max %.2f | processing (frame to LED) ms: mean %.3f max %.3f\n",
		       stat_gn_sum / stat_gn_n, stat_gn_max, stat_proc_sum / stat_gn_n, stat_proc_max);
	}
}

/* ---------------- Main ---------------- */

int main(int argc, char *argv[])
{
	int tcp_port = DEF_TCP_PORT;
	if (argc > 1)
	{
		udp_port = atoi(argv[1]);
	}
	if (argc > 2)
	{
		tcp_port = atoi(argv[2]);
	}
	if (argc > 3)
	{
		log_path = argv[3];
	}

	for (int i = 0; i < MAX_CLIENTS; i++)
	{
		clients[i].fd = -1;
	}
	signal(SIGINT, handle_stop_signal);
	signal(SIGTERM, handle_stop_signal);
	signal(SIGPIPE, SIG_IGN);
	setvbuf(stdout, NULL, _IOLBF, 0);

	udp_fd        = socket(AF_INET, SOCK_DGRAM, 0);
	tcp_listen_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (udp_fd < 0 || tcp_listen_fd < 0)
	{
		perror("socket");
		return EXIT_FAILURE;
	}
	int one = 1;
	setsockopt(tcp_listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
	struct sockaddr_in la;
	memset(&la, 0, sizeof(la));
	la.sin_family      = AF_INET;
	la.sin_addr.s_addr = htonl(INADDR_ANY);
	la.sin_port        = htons((uint16_t)tcp_port);
	if (bind(tcp_listen_fd, (struct sockaddr *)&la, sizeof(la)) < 0 || listen(tcp_listen_fd, MAX_CLIENTS) < 0)
	{
		perror("control port");
		return EXIT_FAILURE;
	}
	fcntl(tcp_listen_fd, F_SETFL, O_NONBLOCK);

	if (argc > 4)
	{
		memset(&viewer_addr, 0, sizeof(viewer_addr));
		viewer_addr.sin_family = AF_INET;
		viewer_addr.sin_port   = htons((uint16_t)udp_port);
		viewer_set             = inet_pton(AF_INET, argv[4], &viewer_addr.sin_addr) == 1;
	}

	if (!airburst_init())
	{
		return EXIT_FAILURE;
	}
	read_library_defaults();
	radar_defaults(&radar_active);
	radar_pending = radar_active;
	live_defaults(&live);
	if (argc > 3)
	{
		live.log_on = 1;
		open_log();
	}

	running = true; /* measure from the start, like the example; the UI can stop it */
	if (!create_detector())
	{
		/* Keep running: the main loop retries every RESTART_RETRY_MS (e.g. while the sensor is reseated). */
		printf("Sensor not available yet - retrying every %.0f ms\n", RESTART_RETRY_MS);
		next_retry_ms = now_ms() + RESTART_RETRY_MS;
	}
	t0_ms = now_ms();
	printf("trigger_exe: range %.2f-%.2f m, %.0f Hz, threshold %.2f / re-arm %.2f. Control TCP %d, stream UDP %d\n",
	       (double)radar_active.start_m, (double)(radar_active.start_m + radar_active.length_m),
	       (double)radar_active.rate_hz, (double)live.trigger_thr, (double)live.rearm_thr, tcp_port, udp_port);

	struct timespec next;
	clock_gettime(CLOCK_MONOTONIC, &next);
	while (!stop_requested)
	{
		poll_control();

		if (led_state && now_ms() >= led_off_at_ms)
		{
			led_set(false);
		}
		if (!running)
		{
			usleep(10000);
			clock_gettime(CLOCK_MONOTONIC, &next);
			continue;
		}
		if (!sensor_ok)
		{
			try_restart("sensor not available");
			usleep(10000);
			clock_gettime(CLOCK_MONOTONIC, &next);
			continue;
		}

		acc_detector_presence_result_t res;
		uint16_t                       n   = 0;
		float                          *vec = NULL;
		double                         tb  = now_ms();
		bool ok = acc_detector_presence_distance_point_vector_get_next(handle, &n, &vec, &res);
		double ta = now_ms();
		if (!ok || res.sensor_communication_error)
		{
			sensor_ok = false;
			try_restart(ok ? "communication error" : "get_next failed");
			continue;
		}
		process_frame(&res, vec, n, tb, ta);

		/* Host pacing at the configured rate (the detector expects the client to keep the timing). */
		long period_ns = (long)(1e9 / (double)radar_active.rate_hz);
		next.tv_nsec += period_ns;
		while (next.tv_nsec >= 1000000000L)
		{
			next.tv_nsec -= 1000000000L;
			next.tv_sec++;
		}
		struct timespec cur;
		clock_gettime(CLOCK_MONOTONIC, &cur);
		double late_ns = (double)(cur.tv_sec - next.tv_sec) * 1e9 + (double)(cur.tv_nsec - next.tv_nsec);
		if (late_ns > (double)period_ns)
		{
			next = cur; /* fell behind by more than a frame: resync (counted as missed by the interval check) */
		}
		else
		{
			clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
		}
		if (status_msg[0] != '\0')
		{
			printf("%s\n", status_msg);
			status_msg[0] = '\0';
		}
	}

	led_set(false);
	acc_libgpiod_set(LED_PIN, PIN_LOW);
	destroy_detector();
	close_log();
	print_summary();
	airburst_destroy();
	return EXIT_SUCCESS;
}
