/*
 * distance_frames.c - A111 distance detector, streaming every frame's full envelope to the PC, with remote
 * control of the radar settings.
 *
 * Acquisition (from Acconeer's example_detector_distance.c, RSS 2.15.4): the distance detector returns
 * peaks only, so the full per-distance amplitude profile is taken from the underlying Envelope service via
 * acc_detector_distance_configuration_service_data_callback_set() ("the unprocessed envelope data that is
 * fed into the distance algorithm"). One frame = one acc_detector_distance_get_next(); if the detector
 * averages several sweeps per result, those sweeps are averaged here too.
 *
 * Defaults = Acconeer's distance detector example in Exploration Tool (exptool a111/algo/distance_detector):
 *   range 0.2-0.6 m, update rate 40 Hz, gain 0.5, and the Envelope defaults it inherits: HWAAS 10,
 *   profile 2, downsampling 1 (~0.484 mm), MUR 6, power save ACTIVE, no running average.
 *   Their GUI plots every sweep at 40 Hz and averages 5 sweeps only for peak finding; here one frame is one
 *   detector result, so sweep averaging is 1 (40 frames/s like their plot). Set sweep_avg to 5 for their
 *   peak averaging (8 frames/s).
 *
 * Radar settings (staged with "set", applied with "apply" = deactivate -> reconfigure -> activate; a
 * rejected set is rolled back to the last working one):
 *   start, end (m), rate (Hz, 0 = as fast as possible; host-paced, the detector has no rate setting),
 *   profile, hwaas, gain, mur (6 or 9; end > 7.0 m needs 9), downsampling (1/2/4), power_save (0-4),
 *   sweep_avg, max_atten (maximize signal attenuation).
 *   raf (running-average factor 0..1 on the envelope, y = raf*y + (1-raf)*x) applies immediately.
 *   Not exposed by the RSS distance detector API: repetition mode, TX disable, noise-level normalization.
 *
 * Control: TCP, one text command per line: start | stop | restart | apply | revert | defaults | get |
 * set <key> <value>. Reply: "ok|err <cmd>[: reason] | key=active/pending ... status".
 * Stream: one UDP packet per frame to the newest control client's IP (or viewer_ip).
 *
 * Usage: distance_exe [viewer_ip] [udp_port] [tcp_port]
 *   viewer_ip  where to stream before a viewer connects (default 192.168.0.123)
 *   udp_port   stream port (default 5008), tcp_port control port (default 5009)
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

#include "acc_detector_distance.h"
#include "acc_version.h"
#include "airburst.h"

/* ---------------- Defaults (Acconeer distance detector example) ---------------- */

#define SENSOR_ID           1
#define DEF_START_M         0.2f
#define DEF_END_M           0.6f
#define DEF_RATE_HZ         40.0f
#define DEF_PROFILE         2
#define DEF_HWAAS           10
#define DEF_GAIN            0.5f
#define DEF_MUR             6
#define DEF_DOWNSAMPLING    1   /* ~0.484 mm */
#define DEF_POWER_SAVE      3   /* ACTIVE */
#define DEF_SWEEP_AVG       1   /* one sweep per frame, as their plot (their peak averaging: 5) */
#define MUR6_MMD_M          7.0f
#define MAX_PEAKS           5

#define MAX_SAMPLES         32000 /* UDP payload limit ~32700 samples */
#define RESTART_RETRY_MS    1000.0
#define MAX_CLIENTS         4

#define DEF_VIEWER_IP       "192.168.0.123"
#define DEF_UDP_PORT        5008
#define DEF_TCP_PORT        5009

#define PACKET_MAGIC        0x46443141u /* "A1DF" */
#define PACKET_VERSION      4u

/* ---------------- Configuration ---------------- */

typedef struct
{
	float start_m;
	float end_m;
	float rate_hz;
	int   profile;
	int   hwaas;
	float gain;
	int   mur;
	int   downsampling;
	int   power_save;
	int   sweep_avg;
	int   max_atten;
	float raf; /* applied immediately (not staged) */
} radar_cfg_t;

typedef enum
{
	K_FLOAT,
	K_INT,
} key_type_t;

typedef struct
{
	const char *name;
	key_type_t  type;
	bool        staged; /* true: takes effect on "apply" */
	size_t      offset;
	double      lo;
	double      hi;
} key_def_t;

#define RK(n, t, f, lo, hi) { n, t, true, offsetof(radar_cfg_t, f), lo, hi }
#define LK(n, t, f, lo, hi) { n, t, false, offsetof(radar_cfg_t, f), lo, hi }

static const key_def_t keys[] = {
	RK("start", K_FLOAT, start_m, 0.0, 12.7),
	RK("end", K_FLOAT, end_m, 0.05, 12.7),
	RK("rate", K_FLOAT, rate_hz, 0.0, 1000.0),
	RK("profile", K_INT, profile, 1, 5),
	RK("hwaas", K_INT, hwaas, 1, 63),
	RK("gain", K_FLOAT, gain, 0.0, 1.0),
	RK("mur", K_INT, mur, 6, 9),
	RK("downsampling", K_INT, downsampling, 1, 4),
	RK("power_save", K_INT, power_save, 0, 4),
	RK("sweep_avg", K_INT, sweep_avg, 1, 100),
	RK("max_atten", K_INT, max_atten, 0, 1),
	LK("raf", K_FLOAT, raf, 0.0, 0.999),
};

#define NKEYS (sizeof(keys) / sizeof(keys[0]))

static radar_cfg_t radar_active;   /* what the detector runs with (last good) */
static radar_cfg_t radar_pending;  /* staged by "set", committed by "apply" */

/* ---------------- State ---------------- */

static volatile sig_atomic_t stop_requested = 0;

static acc_detector_distance_handle_t   handle = NULL;
static acc_detector_distance_metadata_t meta;
static bool   running   = false;
static bool   sensor_ok = false;
static double next_retry_ms = 0.0;
static double last_apply_ms = 0.0;

/* Envelope from the service data callback */
static uint32_t env_sum[MAX_SAMPLES];
static uint16_t env_len   = 0;
static uint16_t env_count = 0;
static float    env_avg[MAX_SAMPLES];   /* running-average state */
static uint16_t avg_len   = 0;

/* Counters */
static uint32_t frame_no = 0, missed_count = 0, restart_count = 0;
static double   t0_ms = 0.0, last_frame_ms = 0.0;
static double   rate_ema = 0.0;

/* I/O */
typedef struct
{
	int    fd;
	char   buf[1024];
	size_t len;
} client_t;

static client_t clients[MAX_CLIENTS];
static int  reply_fd = -1;
static int  udp_fd = -1, tcp_listen_fd = -1;
static int  udp_port = DEF_UDP_PORT;
static struct sockaddr_in viewer_addr;
static bool viewer_set = false;

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

static acc_power_save_mode_t power_save_enum(int v)
{
	static const acc_power_save_mode_t m[] = {ACC_POWER_SAVE_MODE_OFF, ACC_POWER_SAVE_MODE_SLEEP,
	                                          ACC_POWER_SAVE_MODE_READY, ACC_POWER_SAVE_MODE_ACTIVE,
	                                          ACC_POWER_SAVE_MODE_HIBERNATE};
	return m[v < 0 ? 0 : (v > 4 ? 4 : v)];
}

static void envelope_callback(const uint16_t *data, uint16_t data_length)
{
	if (data_length > MAX_SAMPLES)
	{
		data_length = MAX_SAMPLES;
	}
	if (env_count == 0 || data_length != env_len)
	{
		memset(env_sum, 0, sizeof(env_sum));
		env_len   = data_length;
		env_count = 0;
	}
	for (uint16_t i = 0; i < data_length; i++)
	{
		env_sum[i] += data[i];
	}
	env_count++;
}

static void radar_defaults(radar_cfg_t *r)
{
	r->start_m      = DEF_START_M;
	r->end_m        = DEF_END_M;
	r->rate_hz      = DEF_RATE_HZ;
	r->profile      = DEF_PROFILE;
	r->hwaas        = DEF_HWAAS;
	r->gain         = DEF_GAIN;
	r->mur          = DEF_MUR;
	r->downsampling = DEF_DOWNSAMPLING;
	r->power_save   = DEF_POWER_SAVE;
	r->sweep_avg    = DEF_SWEEP_AVG;
	r->max_atten    = 0;
	r->raf          = 0.0f;
}

/* Returns NULL with a reason in err if the settings cannot work. */
static acc_detector_distance_configuration_t build_config(const radar_cfg_t *r, char *err, size_t err_size)
{
	if (r->end_m <= r->start_m + 0.01f)
	{
		snprintf(err, err_size, "end must be above start");
		return NULL;
	}
	if (r->mur != 6 && r->mur != 9)
	{
		snprintf(err, err_size, "mur must be 6 or 9");
		return NULL;
	}
	if (r->mur == 6 && r->end_m > MUR6_MMD_M)
	{
		snprintf(err, err_size, "end above 7.0 m needs mur 9");
		return NULL;
	}
	if (r->downsampling != 1 && r->downsampling != 2 && r->downsampling != 4)
	{
		snprintf(err, err_size, "downsampling must be 1, 2 or 4");
		return NULL;
	}
	acc_detector_distance_configuration_t c = acc_detector_distance_configuration_create();
	if (c == NULL)
	{
		snprintf(err, err_size, "configuration_create failed");
		return NULL;
	}
	acc_detector_distance_configuration_sensor_set(c, SENSOR_ID);
	acc_detector_distance_configuration_requested_start_set(c, r->start_m);
	acc_detector_distance_configuration_requested_length_set(c, r->end_m - r->start_m);
	acc_detector_distance_configuration_service_profile_set(c, (acc_service_profile_t)r->profile);
	acc_detector_distance_configuration_hw_accelerated_average_samples_set(c, (uint8_t)r->hwaas);
	acc_detector_distance_configuration_receiver_gain_set(c, r->gain);
	acc_detector_distance_configuration_mur_set(c, r->mur == 9 ? ACC_SERVICE_MUR_9 : ACC_SERVICE_MUR_6);
	acc_detector_distance_configuration_downsampling_factor_set(c, (uint16_t)r->downsampling);
	acc_detector_distance_configuration_power_save_mode_set(c, power_save_enum(r->power_save));
	acc_detector_distance_configuration_sweep_averaging_set(c, (uint16_t)r->sweep_avg);
	acc_detector_distance_configuration_maximize_signal_attenuation_set(c, r->max_atten != 0);
	acc_detector_distance_configuration_service_data_callback_set(c, envelope_callback);
	return c;
}

static void reset_processing(void)
{
	avg_len       = 0;
	last_frame_ms = 0.0;
	rate_ema      = 0.0;
}

static void destroy_detector(void)
{
	if (handle != NULL)
	{
		acc_detector_distance_deactivate(handle);
		acc_detector_distance_destroy(&handle);
	}
	sensor_ok = false;
}

static bool create_detector(char *err, size_t err_size)
{
	acc_detector_distance_configuration_t c = build_config(&radar_active, err, err_size);
	if (c == NULL)
	{
		return false;
	}
	handle = acc_detector_distance_create(c);
	acc_detector_distance_configuration_destroy(&c);
	if (handle == NULL)
	{
		snprintf(err, err_size, "detector create failed (sensor not answering or unsupported settings)");
		return false;
	}
	if (!acc_detector_distance_metadata_get(handle, &meta) || (running && !acc_detector_distance_activate(handle)))
	{
		snprintf(err, err_size, "metadata/activate failed");
		acc_detector_distance_destroy(&handle);
		return false;
	}
	sensor_ok = true;
	reset_processing();
	return true;
}

static void try_restart(const char *why)
{
	double t = now_ms();
	char   err[160];
	if (t < next_retry_ms)
	{
		return;
	}
	next_retry_ms = t + RESTART_RETRY_MS;
	restart_count++;
	printf("Sensor restart #%u (%s)\n", (unsigned)restart_count, why);
	destroy_detector();
	if (create_detector(err, sizeof(err)))
	{
		return;
	}
	airburst_destroy();
	if (!(airburst_init() && create_detector(err, sizeof(err))))
	{
		printf("Restart failed: %s - retrying\n", err);
	}
}

/* ---------------- Commands ---------------- */

static double key_value(const key_def_t *k, const radar_cfg_t *r)
{
	const uint8_t *p = (const uint8_t *)r + k->offset;
	return k->type == K_FLOAT ? (double)*(const float *)(const void *)p : (double)*(const int *)(const void *)p;
}

static int format_cfg(char *buf, size_t size)
{
	int n = 0;
	for (size_t i = 0; i < NKEYS && n < (int)size; i++)
	{
		const key_def_t *k = &keys[i];
		if (k->staged)
		{
			n += snprintf(buf + n, size - (size_t)n, " %s=%g/%g", k->name, key_value(k, &radar_active),
			              key_value(k, &radar_pending));
		}
		else
		{
			n += snprintf(buf + n, size - (size_t)n, " %s=%g", k->name, key_value(k, &radar_active));
		}
	}
	if (n < (int)size)
	{
		float step = (sensor_ok && env_len > 1) ? meta.length_m / (float)(env_len - 1) : 0.0f;
		n += snprintf(buf + n, size - (size_t)n,
		              " running=%d sensor_ok=%d apply_ms=%.1f samples=%u step_mm=%.4f range_m=%.4f-%.4f rss=%s",
		              running ? 1 : 0, sensor_ok ? 1 : 0, last_apply_ms, (unsigned)env_len, (double)(step * 1000.0f),
		              (double)meta.start_m, (double)(meta.start_m + meta.length_m), acc_version_get());
	}
	return n;
}

static bool apply_radar(char *err, size_t err_size)
{
	double t = now_ms();
	if (handle != NULL && running)
	{
		acc_detector_distance_deactivate(handle);
	}
	acc_detector_distance_configuration_t c = build_config(&radar_pending, err, err_size);
	bool ok = c != NULL && handle != NULL && acc_detector_distance_reconfigure(&handle, c);
	if (c != NULL)
	{
		acc_detector_distance_configuration_destroy(&c);
		if (!ok && err[0] == '\0')
		{
			snprintf(err, err_size, "settings rejected by the detector");
		}
	}
	ok = ok && acc_detector_distance_metadata_get(handle, &meta);
	if (ok && running && !acc_detector_distance_activate(handle))
	{
		snprintf(err, err_size, "activate failed");
		ok = false;
	}
	if (ok)
	{
		float raf    = radar_active.raf;
		radar_active = radar_pending;
		radar_active.raf = raf;
		sensor_ok    = true;
		reset_processing();
	}
	else
	{
		/* Roll back: rebuild with the last working settings. */
		char err2[160];
		radar_pending = radar_active;
		destroy_detector();
		if (!create_detector(err2, sizeof(err2)))
		{
			try_restart("rejected apply");
		}
	}
	last_apply_ms = now_ms() - t;
	return ok;
}

static void full_restart(char *err, size_t err_size)
{
	destroy_detector();
	airburst_destroy();
	frame_no = missed_count = restart_count = 0;
	radar_pending = radar_active;
	running       = true;
	t0_ms         = now_ms();
	next_retry_ms = 0.0;
	if (!(airburst_init() && create_detector(err, err_size)))
	{
		next_retry_ms = now_ms() + RESTART_RETRY_MS;
	}
}

static void process_command(char *line)
{
	static char reply[4096];
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
			if (handle != NULL && acc_detector_distance_activate(handle))
			{
				sensor_ok = true;
				reset_processing();
			}
			else
			{
				destroy_detector();
				if (!create_detector(err, sizeof(err)))
				{
					ok            = false;
					next_retry_ms = 0.0;
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
				acc_detector_distance_deactivate(handle);
			}
		}
	}
	else if (strcmp(cmd, "restart") == 0)
	{
		full_restart(err, sizeof(err));
		ok = sensor_ok;
	}
	else if (strcmp(cmd, "apply") == 0)
	{
		ok = apply_radar(err, sizeof(err));
	}
	else if (strcmp(cmd, "revert") == 0)
	{
		radar_pending = radar_active;
	}
	else if (strcmp(cmd, "defaults") == 0)
	{
		radar_defaults(&radar_pending);
		radar_active.raf = radar_pending.raf;
	}
	else if (strcmp(cmd, "get") == 0)
	{
	}
	else if (strcmp(cmd, "set") == 0)
	{
		char *name = strtok_r(NULL, " \t\r\n", &save);
		char *val  = strtok_r(NULL, " \t\r\n", &save);
		const key_def_t *k = NULL;
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
			/* staged keys go to pending; immediate keys (raf) to both */
			radar_cfg_t *targets[2] = {&radar_pending, k->staged ? NULL : &radar_active};
			for (int i = 0; i < 2; i++)
			{
				if (targets[i] == NULL)
				{
					continue;
				}
				uint8_t *p = (uint8_t *)targets[i] + k->offset;
				if (k->type == K_FLOAT)
				{
					*(float *)(void *)p = (float)v;
				}
				else
				{
					*(int *)(void *)p = (int)lround(v);
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
		(void)send(reply_fd, reply, (size_t)n, MSG_NOSIGNAL);
	}
}

static void poll_control(void)
{
	struct sockaddr_in peer;
	socklen_t          plen = sizeof(peer);
	int                fd   = accept(tcp_listen_fd, (struct sockaddr *)&peer, &plen);
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
			close(fd);
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
			c->len = 0;
		}
	}
}

/* ---------------- Frame processing ---------------- */

static uint8_t *put(uint8_t *p, const void *v, size_t size)
{
	memcpy(p, v, size);
	return p + size;
}

static void process_frame(const acc_detector_distance_result_t *peaks, const acc_detector_distance_result_info_t *info,
                          float get_next_ms)
{
	static uint8_t  packet[128 + 2 * MAX_SAMPLES];
	static uint16_t out[MAX_SAMPLES];
	double t        = now_ms();
	double interval = last_frame_ms > 0.0 ? t - last_frame_ms : 0.0;
	last_frame_ms   = t;
	frame_no++;
	if (interval > 0.0)
	{
		double inst = 1000.0 / interval;
		rate_ema    = rate_ema > 0.0 ? 0.9 * rate_ema + 0.1 * inst : inst;
		if (radar_active.rate_hz > 0.0f && interval > 1.5 * 1000.0 / (double)radar_active.rate_hz)
		{
			missed_count += (uint32_t)(interval * (double)radar_active.rate_hz / 1000.0 + 0.5) - 1u;
		}
	}
	if (info->missed_data)
	{
		missed_count++;
	}

	uint16_t n    = env_len;
	float    step = n > 1 ? meta.length_m / (float)(n - 1) : 0.0f;

	/* Sweep average -> running average (raf) -> output envelope */
	float raf  = radar_active.raf;
	bool  keep = raf > 0.0f && avg_len == n;
	for (uint16_t i = 0; i < n; i++)
	{
		float x    = (float)env_sum[i] / (float)env_count;
		env_avg[i] = keep ? raf * env_avg[i] + (1.0f - raf) * x : x;
		float o    = env_avg[i] + 0.5f;
		out[i]     = (uint16_t)(o > 65535.0f ? 65535.0f : o);
	}
	avg_len = n;

	if (!viewer_set)
	{
		return;
	}
	uint16_t npk   = info->number_of_peaks > MAX_PEAKS ? MAX_PEAKS : info->number_of_peaks;
	uint8_t *p     = packet;
	uint32_t magic = PACKET_MAGIC;
	uint16_t ver   = PACKET_VERSION;
	uint16_t flags = (uint16_t)((info->missed_data ? 1u : 0u) | (info->data_saturated ? 2u : 0u) |
	                            (info->data_quality_warning ? 4u : 0u) | (running ? 8u : 0u));
	float f_t = (float)((t - t0_ms) / 1000.0), f_int = (float)interval, f_rate = (float)rate_ema;
	float f_apply = (float)last_apply_ms;
	p = put(p, &magic, 4);
	p = put(p, &ver, 2);
	p = put(p, &n, 2);
	p = put(p, &frame_no, 4);
	p = put(p, &flags, 2);
	p = put(p, &npk, 2);
	p = put(p, &f_t, 4);
	p = put(p, &f_int, 4);
	p = put(p, &meta.start_m, 4);
	p = put(p, &step, 4);
	p = put(p, &f_rate, 4);
	p = put(p, &f_apply, 4);
	p = put(p, &get_next_ms, 4); /* time the sensor took to deliver this frame */
	p = put(p, &missed_count, 4);
	p = put(p, &restart_count, 4);
	for (uint16_t k = 0; k < MAX_PEAKS; k++) /* 5 slots: distance (m), amplitude */
	{
		float    d = k < npk ? peaks[k].distance_m : 0.0f;
		uint16_t a = k < npk ? peaks[k].amplitude : 0u;
		p = put(p, &d, 4);
		p = put(p, &a, 2);
	}                                         /* header = 82 bytes */
	p = put(p, out, 2u * n);
	sendto(udp_fd, packet, (size_t)(p - packet), MSG_DONTWAIT, (const struct sockaddr *)&viewer_addr,
	       sizeof(viewer_addr));
}

/* ---------------- Main ---------------- */

int main(int argc, char *argv[])
{
	const char *viewer_ip = argc > 1 ? argv[1] : DEF_VIEWER_IP;
	int         tcp_port  = argc > 3 ? atoi(argv[3]) : DEF_TCP_PORT;
	if (argc > 2)
	{
		udp_port = atoi(argv[2]);
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
	memset(&viewer_addr, 0, sizeof(viewer_addr));
	viewer_addr.sin_family = AF_INET;
	viewer_addr.sin_port   = htons((uint16_t)udp_port);
	viewer_set             = inet_pton(AF_INET, viewer_ip, &viewer_addr.sin_addr) == 1;

	if (!airburst_init())
	{
		return EXIT_FAILURE;
	}
	radar_defaults(&radar_active);
	radar_pending = radar_active;

	char err[160] = "";
	running       = true;
	t0_ms         = now_ms();
	if (!create_detector(err, sizeof(err)))
	{
		printf("Sensor not available yet (%s) - retrying every %.0f ms\n", err, RESTART_RETRY_MS);
		next_retry_ms = now_ms() + RESTART_RETRY_MS;
	}
	printf("distance_exe (RSS %s): range %.3f-%.3f m, %.0f Hz, profile %d, HWAAS %d, MUR %d, downsampling %d, "
	       "sweep avg %d; control TCP %d, stream UDP %d -> %s\n",
	       acc_version_get(), (double)meta.start_m, (double)(meta.start_m + meta.length_m),
	       (double)radar_active.rate_hz, radar_active.profile, radar_active.hwaas, radar_active.mur,
	       radar_active.downsampling, radar_active.sweep_avg, tcp_port, udp_port, viewer_ip);

	acc_detector_distance_result_t      peaks[MAX_PEAKS];
	acc_detector_distance_result_info_t info;
	struct timespec                     next;
	clock_gettime(CLOCK_MONOTONIC, &next);
	bool printed_layout = false;

	while (!stop_requested)
	{
		poll_control();
		if (!running || !sensor_ok)
		{
			if (running)
			{
				try_restart("sensor not available");
			}
			usleep(10000);
			clock_gettime(CLOCK_MONOTONIC, &next);
			continue;
		}
		env_count = 0;
		double t_gn = now_ms();
		bool   got  = acc_detector_distance_get_next(handle, peaks, MAX_PEAKS, &info);
		float  gn   = (float)(now_ms() - t_gn);
		if (!got || info.sensor_communication_error)
		{
			sensor_ok = false;
			continue;
		}
		if (env_count == 0 || env_len == 0)
		{
			continue;
		}
		if (!printed_layout)
		{
			printf("Envelope: %u samples per frame, %.3f mm apart\n", (unsigned)env_len,
			       (double)(meta.length_m / (float)(env_len - 1) * 1000.0f));
			printed_layout = true;
		}
		process_frame(peaks, &info, gn);

		if (radar_active.rate_hz > 0.0f) /* host pacing of the requested update rate */
		{
			long period_ns = (long)(1e9 / (double)radar_active.rate_hz);
			next.tv_nsec += period_ns;
			while (next.tv_nsec >= 1000000000L)
			{
				next.tv_nsec -= 1000000000L;
				next.tv_sec++;
			}
			struct timespec cur;
			clock_gettime(CLOCK_MONOTONIC, &cur);
			double late = (double)(cur.tv_sec - next.tv_sec) * 1e9 + (double)(cur.tv_nsec - next.tv_nsec);
			if (late > (double)period_ns)
			{
				next = cur;
			}
			else
			{
				/* Wait for the next frame time in short steps, answering commands meanwhile. */
				for (;;)
				{
					clock_gettime(CLOCK_MONOTONIC, &cur);
					double left_ns = (double)(next.tv_sec - cur.tv_sec) * 1e9 + (double)(next.tv_nsec - cur.tv_nsec);
					if (left_ns <= 0.0)
					{
						break;
					}
					poll_control();
					if (!running || !sensor_ok)
					{
						break;
					}
					usleep((useconds_t)(left_ns > 5e6 ? 5000.0 : left_ns / 1000.0));
				}
			}
		}
		else
		{
			clock_gettime(CLOCK_MONOTONIC, &next);
		}
	}

	destroy_detector();
	airburst_destroy();
	printf("Stopped after %u frames; missed %u, restarts %u\n", (unsigned)frame_no, (unsigned)missed_count,
	       (unsigned)restart_count);
	return EXIT_SUCCESS;
}
