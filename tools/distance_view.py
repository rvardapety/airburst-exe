#!/usr/bin/env python3
"""A111 distance viewer - desktop interface for distance_exe (src/distance_frames.c).

Left sidebar (collapsible sections): connection, Start/Stop/Restart, radar settings (staged on the Pi,
yellow until "Apply radar settings"), advanced sensor settings, scan controls (service, recording
buffer, save/load), display settings.

Right: toolbar (Live, Record, status, Reset zoom), measurement info, and three plots:
  1. Current frame  - envelope amplitude vs distance (all measured samples), vertical guide at the
                      selected distance, detector peaks.
  2. Waterfall      - one row per frame, newest at the top; draggable distance marker.
  3. Amplitude history at the selected distance, over every frame shown in the waterfall.
A slider + numeric readout between the waterfall and the history select the distance (snapped to the
nearest measured sample); slider, marker and guide stay synchronized and the history redraws at once,
also when stopped, paused or replaying.
Zoom/pan: mouse wheel zooms the distance axis around the cursor, drag pans; current frame and waterfall
share the distance axis; the history has its own zoom/pan; "Reset zoom" restores the full range.
The view is kept as new frames arrive; zoom only changes the drawing, never acquisition or stored data.

Recording: "Record" stores every received frame (full envelope) up to "Max buffered frames"; recordings
can be saved (.npz), loaded and replayed with Play/Pause, position slider and speed; all plots, marker
and zoom work during replay.

Everything shown comes from the radar (live or recorded); nothing is simulated.

Run on the PC:
    ~/acconeer-a111/bin/python tools/distance_view.py
"""
import collections
import json
import re
import subprocess
import socket
import struct
import sys
import threading
import time

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

DEFAULT_PI_IP = "192.168.0.124"
UDP_PORT = 5008
TCP_PORT = 5009
LIVE_RING = 1000          # frames kept for the live waterfall (display window <= this)
DEFAULT_WATERFALL = 50
DEFAULT_MAX_BUFFER = 1000  # recording cap (~48 MB at 24,365 samples)
DEFAULT_SELECTED_MM = 1500.0
MAX_UI_FPS = 30           # redraw cap: smooth, and leaves time for mouse/keyboard input
MAX_IMG_COLS = 1024       # waterfall drawn at most this wide (peak-preserving); data keeps every sample
DEFAULT_AMP_MAX = 3000    # fixed amplitude: current-frame axis and waterfall colours (0 = auto)
AMP_SLIDER_MAX = 20000    # slider range (the number boxes go to 65535)
AMP_SLIDER_STEP = 100

MAGIC = 0x46443141
HEADER = struct.Struct("<IHHIHHfffffffII" + "fH" * 5)  # 82 bytes, distance_frames.c version 4
F_MISSED, F_SAT, F_QUALITY, F_RUNNING = 1, 2, 4, 8

# ---------------------------------------------------------------- data


def parse(packet):
    if len(packet) < HEADER.size:
        return None
    v = HEADER.unpack_from(packet)
    if v[0] != MAGIC or v[1] != 4:
        return None
    n = v[2]
    if len(packet) < HEADER.size + 2 * n:
        return None
    npk = v[5]
    peaks = [(v[15 + 2 * k], v[16 + 2 * k]) for k in range(min(npk, 5))]
    env = np.frombuffer(packet, dtype="<u2", count=n, offset=HEADER.size)  # view on the packet, no copy
    return dict(n=n, frame=v[3], flags=v[4], t=v[6], interval=v[7], start=v[8], step=v[9], rate=v[10],
                apply_ms=v[11], get_next=v[12], missed=v[13], restarts=v[14], peaks=peaks, env=env)


DEFAULT_MTI_TC = 0.5   # s: MTI background time constant


class MTI:
    """Moving target indicator (PC side, does not change the radar or the stored data).
    Keeps a slowly updated background of every distance point and gives |frame - background|:
    still objects (walls, furniture) fade out, things that move or change stand out.
    The time constant is how long a new still object takes to fade into the background."""

    def __init__(self, tc=DEFAULT_MTI_TC):
        self.tc = tc
        self.reset()

    def reset(self):
        self.bg = None
        self.t = None

    def process(self, f):
        env = f["env"].astype(np.float32)
        if self.bg is None or self.bg.shape != env.shape:
            self.bg = env.copy()
            self.t = f["t"]
            return np.zeros_like(env)
        dt = f["t"] - self.t
        self.t = f["t"]
        if not 0.0 < dt < 5.0:
            dt = 0.02
        a = 1.0 - np.exp(-dt / max(self.tc, 1e-3))
        out = np.abs(env - self.bg)
        self.bg += a * (env - self.bg)
        return out


REORDER_WINDOW = 64     # frames: an older frame number within this window is a late/duplicate UDP packet
PAUSE_FACTOR = 4.0      # a Pi frame interval this many times the usual one (and >= PAUSE_MIN_MS) = acquisition pause
PAUSE_MIN_MS = 100.0


class FrameDiff:
    """Frame-to-frame MTI: residual[k] = |current envelope[k] - previous envelope[k]| for two consecutive acquired
    frames on the same range grid. The first frame after a reset only becomes the reference (no residual)."""

    def __init__(self):
        self.ref = None
        self.grid = None

    def reset(self):
        self.ref = None

    def process(self, f, grid):
        cur = f["env"].astype(np.int32)              # int32: a decrease cannot wrap around (uint16)
        out = None
        if self.ref is not None and grid == self.grid:
            out = np.abs(cur - self.ref).astype(np.float32)   # exact (values < 65536)
        self.ref, self.grid = cur, grid
        return out


class Segmenter:
    """Decides, frame by frame, whether a frame continues the stream or starts a new segment (reference reset),
    and whether it is a late/duplicate UDP packet that must be ignored. Uses only what the packets carry plus
    resets the viewer itself requests (commands it sent and saw accepted)."""

    def __init__(self):
        self.last = None
        self.int_ema = None
        self.dt_ema = None                  # usual spacing of the Pi timestamps (s)
        self.expect_new_session = False

    def classify(self, f, grid):
        """Returns (action, reason): action is 'drop', 'session' (new stream session), 'reset' or 'ok'."""
        last = self.last
        if last is None:
            return "session", "first frame"
        if f["frame"] <= last["frame"]:
            new = (self.expect_new_session or f["restarts"] < last["restarts"] or f["t"] < last["t"] - 1.0
                   or last["frame"] - f["frame"] >= REORDER_WINDOW)
            if not new:
                return "drop", "duplicate / out-of-order packet"
            return "session", "distance_exe restarted (frame numbers restarted)"
        if grid != last["grid"]:
            return "session", "range grid changed"
        if f["frame"] != last["frame"] + 1:
            return "reset", f"frame gap ({f['frame'] - last['frame'] - 1} not received)"
        if f["restarts"] != last["restarts"]:
            return "reset", "sensor restart"
        if f["apply_ms"] != last["apply_ms"]:
            return "reset", "radar settings applied"
        if f["missed"] > last["missed"]:
            return "reset", "sensor missed data"
        if f["interval"] <= 0.0:            # distance_exe zeroes it when it restarts processing (start, apply)
            return "reset", "acquisition restarted on the Pi"
        if self.int_ema is not None and f["interval"] > max(PAUSE_MIN_MS, PAUSE_FACTOR * self.int_ema):
            return "reset", f"acquisition pause ({f['interval']:.0f} ms)"
        dt = f["t"] - last["t"]
        if self.dt_ema is not None and dt > max(PAUSE_MIN_MS / 1000.0, PAUSE_FACTOR * self.dt_ema):
            return "reset", f"acquisition pause ({1000 * dt:.0f} ms)"
        return "ok", ""

    def accept(self, f, grid, action):
        if action in ("session", "reset"):
            self.int_ema = self.dt_ema = None
        else:
            if f["interval"] > 0:
                self.int_ema = f["interval"] if self.int_ema is None else 0.9 * self.int_ema + 0.1 * f["interval"]
            dt = f["t"] - self.last["t"]
            if dt > 0:
                self.dt_ema = dt if self.dt_ema is None else 0.9 * self.dt_ema + 0.1 * dt
        self.expect_new_session = False
        self.last = dict(frame=f["frame"], t=f["t"], restarts=f["restarts"], apply_ms=f["apply_ms"],
                         missed=f["missed"], grid=grid)


def recording_segments(z, frames):
    """Segment starts for a recording: stored ones (new files) or derived from frame numbers and time gaps."""
    if "seg_start" in z.files:
        return [bool(x) for x in z["seg_start"]]
    t = np.array([f["t"] for f in frames])
    dt = np.diff(t)
    usual = float(np.median(dt)) if dt.size else 0.0
    out = [True]
    for i in range(1, len(frames)):
        gap = frames[i]["frame"] != frames[i - 1]["frame"] + 1
        pause = usual > 0 and dt[i - 1] > max(PAUSE_MIN_MS / 1000.0, PAUSE_FACTOR * usual)
        out.append(gap or pause)
    return out


class Layout:
    """Measured sample distances (from the start and spacing the Pi reports)."""

    def __init__(self, n, start_m, step_m):
        self.key = (n, round(start_m, 7), round(step_m, 9))
        self.n = n
        self.start_mm = start_m * 1000.0
        self.step_mm = step_m * 1000.0
        self.x_mm = self.start_mm + np.arange(n) * self.step_mm
        self.end_mm = float(self.x_mm[-1])

    def nearest(self, mm):
        i = int(round((mm - self.start_mm) / self.step_mm)) if self.step_mm > 0 else 0
        return min(max(i, 0), self.n - 1)


class Recording:
    def __init__(self, name, frames):
        self.name = name
        self.frames = frames  # list of frame dicts (same layout)

    def label(self):
        f = self.frames
        dur = f[-1]["t"] - f[0]["t"] if len(f) > 1 else 0.0
        return f"{self.name}  ({len(f)} frames, {dur:.1f} s)"

    def save(self, path, settings):
        f = self.frames
        np.savez_compressed(
            path, env=np.stack([x["env"] for x in f]), frame=np.array([x["frame"] for x in f]),
            t=np.array([x["t"] for x in f]), flags=np.array([x["flags"] for x in f]),
            peaks_d=np.array([[p[0] for p in x["peaks"]] + [0.0] * (5 - len(x["peaks"])) for x in f]),
            peaks_a=np.array([[p[1] for p in x["peaks"]] + [0] * (5 - len(x["peaks"])) for x in f]),
            npk=np.array([len(x["peaks"]) for x in f]), start=f[0]["start"], step=f[0]["step"],
            seg_start=np.array([i == 0 or bool(x.get("seg_start", False)) for i, x in enumerate(f)]),
            settings=json.dumps(settings))

    @staticmethod
    def load(path, tc=DEFAULT_MTI_TC):
        z = np.load(path, allow_pickle=False)
        frames = []
        for i in range(len(z["frame"])):
            npk = int(z["npk"][i])
            frames.append(dict(
                n=z["env"].shape[1], frame=int(z["frame"][i]), flags=int(z["flags"][i]), t=float(z["t"][i]),
                interval=0.0, start=float(z["start"]), step=float(z["step"]), rate=0.0, apply_ms=0.0,
                get_next=0.0, missed=0, restarts=0,
                peaks=[(float(z["peaks_d"][i][k]), int(z["peaks_a"][i][k])) for k in range(npk)],
                env=z["env"][i]))
        m = MTI(tc)
        ff = FrameDiff()
        grid = (frames[0]["n"], frames[0]["start"], frames[0]["step"]) if frames else None
        for f, seg in zip(frames, recording_segments(z, frames)):
            f["mti"] = m.process(f)
            if seg:
                ff.reset()
            f["seg_start"] = seg
            f["mti_ff"] = ff.process(f, grid)
        name = path.rsplit("/", 1)[-1]
        return Recording(name, frames)


class Receiver(threading.Thread):
    """UDP stream: every frame goes into the live ring, and into the recording while recording."""

    def __init__(self, port):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16 * 1024 * 1024)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.5)
        self.lock = threading.Lock()
        self.ring = collections.deque(maxlen=LIVE_RING)
        self.layout = None
        self.version = 0
        self.last_rx = 0.0
        self.times = collections.deque(maxlen=50)
        self.recording = None        # list while recording
        self.record_cap = DEFAULT_MAX_BUFFER
        self.pc_lost = 0                 # frames the Pi sent that this window did not receive
        self.record_stopped_reason = ""
        self.mti = MTI()
        self.ff = FrameDiff()
        self.seg = Segmenter()
        self.reset_at = None             # viewer-requested reference reset: reply time of the accepted command
        self.reset_new_session = False
        self.dropped = 0                 # duplicate / out-of-order packets ignored
        self.ff_resets = collections.Counter()
        self.ff_last_reason = ""

    def mark_reset(self, when, new_session=False):
        """Called (from the control thread) when the Pi accepted a command that changes the acquisition."""
        with self.lock:
            self.reset_at = when
            self.reset_new_session = self.reset_new_session or new_session

    def run(self):
        while True:
            try:
                packet, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            rx_t = time.monotonic()              # arrival on the PC (for the display delay)
            f = parse(packet)
            if f is None:
                continue
            key = (f["n"], round(f["start"], 7), round(f["step"], 9))
            with self.lock:
                viewer_reset = False
                if self.reset_at is not None:    # a command this viewer sent changed the acquisition
                    T = self.reset_at
                    if self.reset_new_session:
                        self.seg.expect_new_session = True
                    after = [x for x in self.ring if x.get("rx_t", 0.0) >= T]
                    if after:                    # frames after the change were already processed: the first one
                        after[0]["mti_ff"] = None    # was compared with a frame from before -> void it
                        after[0]["seg_start"] = True
                        self.ff_resets["viewer command"] += 1
                        self.ff_last_reason = "viewer command"
                    else:
                        viewer_reset = True
                    self.reset_at, self.reset_new_session = None, False
                action, reason = self.seg.classify(f, key)
                if action == "drop":             # late or duplicate packet: never replaces the reference
                    self.dropped += 1
                    continue
                if viewer_reset and action == "ok":
                    action, reason = "reset", "viewer command"
                self.seg.accept(f, key, action)
                if action == "session" or self.layout is None or self.layout.key != key:
                    if self.layout is None or self.layout.key != key or self.ring:
                        self.layout = Layout(f["n"], f["start"], f["step"])  # new configuration or restart
                        self.ring.clear()
                        self.times.clear()
                        self.mti.reset()
                        if self.recording is not None and self.recording:
                            self.record_stopped_reason = "radar configuration changed"
                            self.recording_done = self.recording
                            self.recording = None
                if self.ring and f["frame"] > self.ring[-1]["frame"] + 1:
                    self.pc_lost += f["frame"] - self.ring[-1]["frame"] - 1
                f["mti"] = self.mti.process(f)   # always computed, so the MTI view has full history at once
                if action != "ok":
                    self.ff.reset()
                    self.ff_resets[reason.split(" (")[0]] += 1
                    self.ff_last_reason = reason
                f["seg_start"] = action != "ok"
                f["mti_ff"] = self.ff.process(f, key)   # every received frame, independent of redraws
                f["rx_t"] = rx_t
                self.ring.append(f)
                self.times.append(f["t"])
                if self.recording is not None:
                    self.recording.append(f)
                    if len(self.recording) >= self.record_cap:
                        self.record_stopped_reason = "max buffered frames reached"
                        self.recording_done = self.recording
                        self.recording = None
                self.version += 1
                self.last_rx = time.monotonic()

    def radar_rate(self):
        t = self.times
        return (len(t) - 1) / (t[-1] - t[0]) if len(t) > 2 and t[-1] > t[0] else 0.0

    def snapshot(self, count):
        with self.lock:
            frames = list(self.ring)[-count:]
            return frames, self.layout, self.radar_rate(), self.version

    def peek(self):
        with self.lock:
            return self.version, (time.monotonic() - self.last_rx) if self.last_rx else None


class Control:
    """TCP control link to distance_exe: one command line out, one reply line back."""

    def __init__(self):
        self.sock = None
        self.buf = b""

    def connect(self, ip):
        self.close()
        self.sock = socket.create_connection((ip, TCP_PORT), timeout=3)
        self.sock.settimeout(5)

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock, self.buf = None, b""

    def send(self, line):
        if not self.sock:
            raise OSError("not connected")
        self.sock.sendall((line + "\n").encode())
        while b"\n" not in self.buf:
            chunk = self.sock.recv(8192)
            if not chunk:
                raise OSError("connection closed")
            self.buf += chunk
        reply, self.buf = self.buf.split(b"\n", 1)
        return reply.decode(errors="replace")


class ControlWorker(QtCore.QObject):
    """Runs all TCP control traffic in its own thread, so a slow reply (Wi-Fi) never blocks drawing."""
    replied = QtCore.Signal(str, str)   # command, reply line
    failed = QtCore.Signal(str, str)    # command, error text

    def __init__(self):
        super().__init__()
        self.ctl = Control()

    @QtCore.Slot(str, str)
    def run(self, kind, arg):
        try:
            if kind == "connect":
                self.ctl.connect(arg)
                self.replied.emit("connect", "ok connect |")
            elif kind == "close":
                self.ctl.close()
                self.replied.emit("close", "ok close |")
            else:
                reply = self.ctl.send(arg)
                rx = getattr(self, "rx", None)
                if rx is not None and reply.startswith("ok"):
                    words = arg.split()
                    if words[0] in ("start", "restart", "apply", "defaults") or words[:2] == ["set", "raf"]:
                        rx.mark_reset(time.monotonic(), new_session=words[0] == "restart")
                self.replied.emit(arg, reply)
        except OSError as e:
            if kind != "connect":
                self.ctl.close()
            self.failed.emit(arg if kind == "cmd" else kind, str(e))


def parse_reply(reply):
    head, _, tail = reply.partition("|")
    active, pending, info = {}, {}, {}
    for tok in tail.split():
        if "=" not in tok:
            continue
        k, v = tok.split("=", 1)
        if k in ("rss", "range_m"):
            info[k] = v
            continue
        if "/" in v:
            a, p = v.split("/", 1)
            active[k], pending[k] = float(a), float(p)
        else:
            try:
                active[k] = pending[k] = float(v)
            except ValueError:
                info[k] = v
    return head.strip(), active, pending, info

# ---------------------------------------------------------------- widgets


class NoWheel:
    def wheelEvent(self, e):  # scrolling the sidebar must not change values
        e.ignore()


class DSpin(NoWheel, QtWidgets.QDoubleSpinBox):
    pass


class ISpin(NoWheel, QtWidgets.QSpinBox):
    pass


class Combo(NoWheel, QtWidgets.QComboBox):
    pass


class Section(QtWidgets.QWidget):
    """Collapsible sidebar section."""

    def __init__(self, title, expanded=True):
        super().__init__()
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 2, 0, 2)
        lay.setSpacing(0)
        self.btn = QtWidgets.QToolButton(text=title, checkable=True, checked=expanded)
        self.btn.setStyleSheet("QToolButton{border:none;font-weight:600;padding:3px;}")
        self.btn.setToolButtonStyle(QtCore.Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        self.btn.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored, QtWidgets.QSizePolicy.Policy.Fixed)
        self.body = QtWidgets.QFrame()
        self.body.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.form = QtWidgets.QFormLayout(self.body)
        self.form.setContentsMargins(6, 4, 6, 4)
        self.form.setVerticalSpacing(3)
        self.form.setFieldGrowthPolicy(QtWidgets.QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
        lay.addWidget(self.btn)
        lay.addWidget(self.body)
        self.btn.toggled.connect(self.toggle)
        self.toggle(expanded)

    def toggle(self, on):
        self.btn.setArrowType(QtCore.Qt.ArrowType.DownArrow if on else QtCore.Qt.ArrowType.RightArrow)
        self.body.setVisible(on)

# ---------------------------------------------------------------- main window


# Radar keys (staged on the Pi, applied with "Apply radar settings")
RADAR_KEYS = ("start", "end", "rate", "profile", "hwaas", "gain", "mur", "downsampling", "power_save",
              "sweep_avg", "max_atten")
POWER_SAVE = ["off", "sleep", "ready", "active", "hibernate"]
UNSUPPORTED = "Not exposed by the RSS 2.15.4 distance detector API (only by the Envelope service)."

# Radar presets = the rows of the frame-rate measurement (2026-11-18, this sensor, update rate "max",
# sweep averaging 1, power save active, gain 0.5). Frame rates are the measured values.
PRESET_BASE = dict(rate=0, sweep_avg=1, power_save=3, gain=0.5, max_atten=0)
PRESETS = [
    ("Setting 1: 0.2-12 m, 0.48 mm, HWAAS 10, MUR 9  (~10 fps)",
     dict(start=0.2, end=12.0, downsampling=1, hwaas=10, mur=9, profile=2)),
    ("Setting 2: 0.2-12 m, 0.48 mm, HWAAS 1, MUR 9  (~13 fps)",
     dict(start=0.2, end=12.0, downsampling=1, hwaas=1, mur=9, profile=2)),
    ("Setting 3: 0.2-12 m, 1.94 mm, HWAAS 1, MUR 9  (~57 fps)",
     dict(start=0.2, end=12.0, downsampling=4, hwaas=1, mur=9, profile=2)),
    ("Setting 4: 0.2-6.9 m, 1.94 mm, HWAAS 1, MUR 6  (~121 fps)",
     dict(start=0.2, end=6.9, downsampling=4, hwaas=1, mur=6, profile=2)),
    ("Setting 5: 0.2-3 m, 1.94 mm, HWAAS 1, MUR 6  (~290 fps)",
     dict(start=0.2, end=3.0, downsampling=4, hwaas=1, mur=6, profile=2)),
    ("Setting 6: 0.2-0.6 m, 0.48 mm, HWAAS 10, MUR 6  (~345 fps, Acconeer example)",
     dict(start=0.2, end=0.6, downsampling=1, hwaas=10, mur=6, profile=2)),
    ("Setting 7: 0.2-0.6 m, 1.94 mm, HWAAS 1, MUR 6  (~1020 fps)",
     dict(start=0.2, end=0.6, downsampling=4, hwaas=1, mur=6, profile=2)),
    ("Setting 8: 0.2-0.3 m, 1.94 mm, HWAAS 1, MUR 6  (~1510 fps)",
     dict(start=0.2, end=0.3, downsampling=4, hwaas=1, mur=6, profile=2)),
    ("Setting 9: 0.2-0.3 m, 1.94 mm, HWAAS 1, MUR 6, profile 1  (~1580 fps)",
     dict(start=0.2, end=0.3, downsampling=4, hwaas=1, mur=6, profile=1)),
]


class Window(QtWidgets.QMainWindow):
    request = QtCore.Signal(str, str)   # (kind, argument) -> ControlWorker.run

    def __init__(self, rx):
        super().__init__()
        self.rx = rx
        self.connected = False
        self.inflight = 0
        self.worker = ControlWorker()
        self.worker.rx = rx
        self.ctl_thread = QtCore.QThread(self)
        self.worker.moveToThread(self.ctl_thread)
        self.request.connect(self.worker.run)
        self.worker.replied.connect(self.on_reply)
        self.worker.failed.connect(self.on_failed)
        self.ctl_thread.start()
        self.net_rtt = None              # ms, ping round-trip to the Pi (background thread, every 2 s)
        self.pi_ip = DEFAULT_PI_IP       # plain copy for the ping thread (Qt widgets are main-thread only)
        threading.Thread(target=self.ping_loop, daemon=True).start()
        self.active, self.pending, self.info = {}, {}, {}
        self.widgets = {}            # key -> (widget, kind)
        self.syncing = False
        self.mode = "live"           # live | replay
        self.mti_on = False
        self.mti_method = "exp"                                # exp | ff
        self.recordings = []
        self.replay_rec = None
        self.replay_pos = 0
        self.disp = None             # (frames, None, layout) currently drawn
        self.layout = None
        self.sel_idx = 0
        self.selected_mm = DEFAULT_SELECTED_MM
        self.drawn_version = -1
        self.redraws = collections.deque(maxlen=40)
        self.level = None
        self.ymax = self.ytop = None
        self.ui_fps = MAX_UI_FPS
        self.view_layout_key = None
        self.setWindowTitle("A111 distance viewer")
        self.build()
        self.status_tick = QtCore.QTimer(timeout=self.poll_settings, interval=1000)
        self.status_tick.start()
        self.timer = QtCore.QTimer(timeout=self.refresh, interval=16)  # ~60 checks/s: new frames shown within ~16 ms
        self.timer.start()
        self.play_timer = QtCore.QTimer(singleShot=True, timeout=self.play_step)

    # ------------------------------------------------------------ layout
    def build(self):
        split = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        self.setCentralWidget(split)
        split.addWidget(self.build_sidebar())
        split.addWidget(self.build_plots())
        split.setStretchFactor(0, 3)
        split.setStretchFactor(1, 7)
        self.split = split
        sb = self.statusBar()
        self.sb_conn = QtWidgets.QLabel("Not connected")
        self.sb_conn.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored, QtWidgets.QSizePolicy.Policy.Fixed)
        sb.addWidget(self.sb_conn, 1)   # rate / missed frames are in the stats block above the plots

    @staticmethod
    def help(form, text):
        lab = QtWidgets.QLabel(text)
        lab.setWordWrap(True)
        lab.setStyleSheet("color:#616161;font-size:8pt;")
        form.addRow(lab)

    def spin(self, key, lo, hi, dec, step, radar, suffix=""):
        w = DSpin() if dec else ISpin()
        w.setRange(lo, hi)
        if dec:
            w.setDecimals(dec)
        w.setSingleStep(step)
        w.setKeyboardTracking(False)
        if suffix:
            w.setSuffix(suffix)
        w.valueChanged.connect(lambda v, k=key: self.send_set(k, v))
        self.widgets[key] = (w, "spin", radar)
        return w

    def combo(self, key, items, values, radar):
        w = Combo()
        w.addItems(items)
        w.setProperty("values", values)
        w.currentIndexChanged.connect(lambda i, k=key, vals=values: self.send_set(k, vals[i]))
        self.widgets[key] = (w, "combo", radar)
        return w

    def check(self, key, radar, text=""):
        w = QtWidgets.QCheckBox(text)
        w.toggled.connect(lambda v, k=key: self.send_set(k, int(v)))
        self.widgets[key] = (w, "check", radar)
        return w

    def build_sidebar(self):
        side = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(side)
        lay.setContentsMargins(6, 6, 6, 6)
        lay.setSpacing(4)

        top = QtWidgets.QFormLayout()
        top.setVerticalSpacing(3)
        self.iface = Combo()
        self.iface.addItems(["Socket (TCP control + UDP stream)"])
        top.addRow("Interface", self.iface)
        self.proto = QtWidgets.QLineEdit(f"distance_exe v2, TCP {TCP_PORT} / UDP {UDP_PORT}")
        self.proto.setReadOnly(True)
        top.addRow("Protocol", self.proto)
        self.ip = QtWidgets.QLineEdit(DEFAULT_PI_IP)
        top.addRow("Raspberry Pi IP", self.ip)
        lay.addLayout(top)
        self.btn_connect = QtWidgets.QPushButton("Connect")
        self.btn_connect.clicked.connect(self.toggle_connect)
        lay.addWidget(self.btn_connect)
        row = QtWidgets.QHBoxLayout()
        for text, cmd, col in (("Start", "start", "#2e7d32"), ("Stop", "stop", "#c62828"), ("Restart", "restart", "#1565c0")):
            b = QtWidgets.QPushButton(text)
            b.setStyleSheet(f"QPushButton{{background:{col};color:white;font-weight:600;padding:5px;}}"
                            f"QPushButton:disabled{{background:#bdbdbd;}}")
            b.clicked.connect(lambda _=False, c=cmd: self.command(c))
            row.addWidget(b)
        lay.addLayout(row)

        # Radar settings
        # ---- Radar configuration, grouped by what each setting controls (with beginner explanations) ----
        def tip(w, text):
            w.setToolTip(text)
            return w

        s = Section("Preset")
        f = s.form
        self.preset = Combo()
        self.preset.addItem("custom")
        for label, _ in PRESETS:
            self.preset.addItem(label)
        self.preset.setToolTip("Choose a measured configuration: all radar settings are set and applied at once")
        self.preset.activated.connect(self.apply_preset)
        f.addRow("Preset", self.preset)
        lay.addWidget(s)

        s = Section("Range")
        f = s.form
        f.addRow("Range start (m)", tip(self.spin("start", 0.0, 12.7, 3, 0.05, True),
                 "Nearest distance measured. Below ~0.2 m the sensor's own transmit leakage dominates."))
        f.addRow("Range end (m)", tip(self.spin("end", 0.05, 12.7, 3, 0.1, True),
                 "Farthest distance measured. Longer = more points = slower frames. Above 7.0 m needs MUR 9."))
        f.addRow("MUR", tip(self.combo("mur", ["6 (max 7.0 m)", "9 (max 12.7 m)"], [6, 9], True),
                 "Maximum unambiguous range: how far pulses may travel before the next one is sent.\n"
                 "6: pulses every 77 ns (13 MHz), range end up to 7.0 m, faster.\n"
                 "9: pulses every 115 ns (8.7 MHz), range end up to 12.7 m, slower.\n"
                 "Objects beyond the MUR can fold back and appear at a wrong, shorter distance."))
        lay.addWidget(s)

        s = Section("Radar signal (hardware)")
        f = s.form
        f.addRow("Profile", tip(self.combo("profile", ["1 (shortest pulse)", "2", "3", "4", "5 (longest pulse)"],
                                           [1, 2, 3, 4, 5], True),
                 "Length of the transmitted pulse.\n"
                 "Short (1-2): sharper peaks, better at telling close objects apart, weaker at long range.\n"
                 "Long (4-5): more energy, sees weak/far objects better, wider peaks."))
        f.addRow("HWAAS", tip(self.spin("hwaas", 1, 63, 0, 1, True),
                 "Hardware averaging: each point is measured this many times and averaged inside the chip.\n"
                 "Higher = less noise (smoother curve), but each frame takes longer."))
        g = QtWidgets.QHBoxLayout()
        gs = self.spin("gain", 0.0, 1.0, 2, 0.01, True)
        gs.setToolTip("Receiver amplification (0-1).\nHigher = bigger amplitudes (weak echoes visible); "
                      "too high = SATURATED (signal clipped near the sensor).")
        self.gain_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.gain_slider.setRange(0, 100)
        self.gain_slider.setToolTip(gs.toolTip())
        self.gain_slider.valueChanged.connect(lambda v: gs.setValue(v / 100.0))
        gs.valueChanged.connect(lambda v: (self.gain_slider.blockSignals(True),
                                           self.gain_slider.setValue(int(round(v * 100))),
                                           self.gain_slider.blockSignals(False)))
        gs.setFixedWidth(64)                 # number box keeps its size; the slider takes the rest
        g.addWidget(gs)
        g.addWidget(self.gain_slider, 1)
        f.addRow("Receiver gain", g)
        f.addRow("Power-save mode", tip(self.combo("power_save", POWER_SAVE, [0, 1, 2, 3, 4], True),
                 "What the chip does between frames. 'active' stays ready (fastest, most power); "
                 "'off'/'sleep'/'hibernate' save power but need time to wake up."))
        f.addRow("Max. signal attenuation", tip(self.check("max_atten", True),
                 "Turns the received signal down to its minimum. Only useful when strong, very close "
                 "reflections saturate the receiver."))
        lay.addWidget(s)

        s = Section("Sampling && timing")
        f = s.form
        f.addRow("Downsampling", tip(self.combo("downsampling", ["1 (~0.48 mm)", "2 (~0.97 mm)", "4 (~1.94 mm)"],
                                                [1, 2, 4], True),
                 "Distance between measured points.\n1: finest detail, most points, slowest.\n"
                 "4: a quarter of the points, about 4x faster, coarser curve."))
        r = self.spin("rate", 0, 1000, 1, 1, True)
        r.setSpecialValueText("max (as fast as possible)")
        r.setToolTip("Frames per second requested (paced by distance_exe). If the sensor cannot measure that fast, "
                     "the stats show RATE NOT REACHED. Lowest value = as fast as possible.")
        f.addRow("Update rate (Hz)", r)
        lay.addWidget(s)

        s = Section("DSP (signal processing)")
        f = s.form
        f.addRow("Sweep averaging", tip(self.spin("sweep_avg", 1, 100, 0, 1, True),
                 "Measure this many sweeps and average them into one frame.\n"
                 "Higher = less noise, but N times fewer frames per second."))
        raf = self.spin("raf", 0.0, 0.99, 2, 0.05, False)
        raf.setToolTip("Running average over frames (applied immediately): new = f x old + (1 - f) x frame.\n"
                       "0 = off; closer to 1 = smoother but slower to react to movement.")
        f.addRow("Running-average factor", raf)
        self.mti_tc = DSpin()
        self.mti_tc.setRange(0.05, 30.0)
        self.mti_tc.setDecimals(2)
        self.mti_tc.setSingleStep(0.1)
        self.mti_tc.setSuffix(" s")
        self.mti_tc.setValue(DEFAULT_MTI_TC)
        self.mti_tc.setKeyboardTracking(False)
        self.mti_tc.setToolTip("MTI (PC side, applied at once): how long an object must stay still\n"
                               "before it fades out. Short = only fast movement; long = slow movement too.")
        self.mti_tc.valueChanged.connect(self.on_mti_tc)
        f.addRow("MTI background time", self.mti_tc)
        lay.addWidget(s)

        apply_box = QtWidgets.QFrame()
        apply_box.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        al = QtWidgets.QVBoxLayout(apply_box)
        al.setContentsMargins(6, 4, 6, 4)
        self.pending_label = QtWidgets.QLabel("")
        self.pending_label.setStyleSheet("color:#e65100;font-weight:600;")
        al.addWidget(self.pending_label)
        b = QtWidgets.QPushButton("Apply radar settings")
        b.setToolTip("Sends the yellow (changed) settings to the radar. Running-average factor applies at once.")
        b.clicked.connect(lambda: self.command("apply"))
        al.addWidget(b)
        lay.addWidget(apply_box)

        # Advanced
        s = Section("Advanced (not available)", expanded=False)
        f = s.form
        rep = Combo()
        rep.addItems(["On demand (fixed by the distance detector)"])
        rep.setEnabled(False)
        rep.setToolTip(UNSUPPORTED)
        f.addRow("Repetition mode", rep)
        for label in ("Noise-level normalization", "Disable TX"):
            c = QtWidgets.QCheckBox()
            c.setEnabled(False)
            c.setToolTip(UNSUPPORTED)
            f.addRow(label, c)
        lay.addWidget(s)

        # Scan controls
        s = Section("Scan controls")
        f = s.form
        svc = Combo()
        svc.addItems(["Distance detector (envelope)"])
        f.addRow("Service", svc)
        self.max_buf = ISpin()
        self.max_buf.setRange(10, 20000)
        self.max_buf.setValue(DEFAULT_MAX_BUFFER)
        self.max_buf.setToolTip("Recording stops automatically at this many frames (~48 kB per frame at 24,365 samples).")
        self.max_buf.valueChanged.connect(lambda v: setattr(self.rx, "record_cap", v))
        f.addRow("Max buffered frames", self.max_buf)
        self.buf_count = QtWidgets.QLineEdit("0")
        self.buf_count.setReadOnly(True)
        f.addRow("Buffered frames", self.buf_count)
        r2 = QtWidgets.QHBoxLayout()
        bs = QtWidgets.QPushButton("Save to file")
        bs.clicked.connect(self.save_file)
        bl = QtWidgets.QPushButton("Load from file")
        bl.clicked.connect(self.load_file)
        r2.addWidget(bs)
        r2.addWidget(bl)
        f.addRow(r2)
        lay.addWidget(s)

        # Display
        s = Section("Display settings")
        self.wf_frames = ISpin()
        self.wf_frames.setRange(5, LIVE_RING)
        self.wf_frames.setValue(DEFAULT_WATERFALL)
        self.wf_frames.valueChanged.connect(self.on_history_frames)
        s.form.addRow("Waterfall frames", self.wf_frames)
        self.amp_side = self.amp_box()
        s.form.addRow("Amplitude max", self.amp_side)
        lay.addWidget(s)
        lay.addStretch(1)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(side)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setMinimumWidth(260)
        # Let every sidebar field shrink to the column width (long combo texts must not push it wider).
        for c in side.findChildren(QtWidgets.QComboBox):
            c.setSizeAdjustPolicy(QtWidgets.QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
            c.setMinimumContentsLength(4)
        for wdg in (side.findChildren(QtWidgets.QAbstractSpinBox) + side.findChildren(QtWidgets.QLineEdit) +
                    side.findChildren(QtWidgets.QComboBox)):
            if wdg.maximumWidth() < 1000:   # fixed-width fields (gain number box) stay as they are
                continue
            pol = wdg.sizePolicy()
            pol.setHorizontalPolicy(QtWidgets.QSizePolicy.Policy.Ignored)  # take the column width, never widen it
            wdg.setSizePolicy(pol)
            wdg.setMinimumWidth(50)
        for b in side.findChildren(QtWidgets.QPushButton):
            b.setMinimumWidth(40)
        return scroll

    def build_plots(self):
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.setSpacing(3)

        tb = QtWidgets.QHBoxLayout()
        self.btn_live = QtWidgets.QPushButton("Live")
        self.btn_live.clicked.connect(self.go_live)
        self.btn_rec = QtWidgets.QPushButton("Record")
        self.btn_rec.setCheckable(True)
        self.btn_rec.toggled.connect(self.toggle_record)
        self.btn_mti = QtWidgets.QPushButton("MTI: off")
        self.btn_mti.setCheckable(True)
        self.btn_mti.setToolTip("Moving target indicator: hides still objects (walls, furniture), "
                                "shows only what moves or changes. Applied on the PC, at once.")
        self.btn_mti.toggled.connect(self.toggle_mti)
        self.mti_sel = QtWidgets.QComboBox()
        self.mti_sel.addItem("Exponential background", "exp")
        self.mti_sel.addItem("Frame-to-frame", "ff")
        self.mti_sel.setToolTip("MTI method.\nExponential background: |frame - slowly updated background|.\n"
                                "Frame-to-frame: |frame - previous frame| (Envelope change), per distance point.")
        self.mti_sel.currentIndexChanged.connect(lambda i: self.set_mti_method(self.mti_sel.itemData(i)))
        self.state = QtWidgets.QLabel("● DISCONNECTED")
        self.state.setStyleSheet("font-weight:700;color:#757575;padding:0 8px;")
        bz = QtWidgets.QPushButton("Reset zoom")
        bz.clicked.connect(self.reset_view)
        for x in (self.btn_live, self.btn_rec, self.btn_mti, self.mti_sel, self.state):
            tb.addWidget(x)
        tb.addStretch(1)
        tb.addWidget(bz)
        lay.addLayout(tb)

        self.info_line = QtWidgets.QLabel("waiting for frames<br>&nbsp;<br>&nbsp;<br>&nbsp;")
        self.info_line.setTextFormat(QtCore.Qt.TextFormat.RichText)
        self.info_line.setStyleSheet("font-family:monospace;padding:2px 4px;background:#fafafa;"
                                     "border:1px solid #e0e0e0;")
        self.info_line.setFixedHeight(self.info_line.fontMetrics().height() * 4 + 14)
        # Text changes every frame: never let it resize the window layout (that forces full relayouts).
        self.info_line.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored, QtWidgets.QSizePolicy.Policy.Fixed)
        lay.addWidget(self.info_line)

        glw = pg.GraphicsLayoutWidget()
        glw.ci.setSpacing(2)
        lay.addWidget(glw, 4)
        self.p_frame = glw.addPlot(row=0, col=0, title="Current frame")
        self.p_frame.setLabel("left", "amplitude")
        self.p_frame.setLabel("bottom", "distance (mm)")
        # Grid: one light line item behind the curve, rebuilt only when the view range changes
        # (the library's built-in grid is re-rendered with the axes on every frame: ~20% fewer frames drawn).
        self.grid = pg.PlotCurveItem(pen=pg.mkPen((0, 0, 0, 40), width=1), connect="pairs")
        self.grid.setZValue(-10)
        self.p_frame.addItem(self.grid, ignoreBounds=True)
        self.p_frame.sigRangeChanged.connect(self.update_grid)
        self.c_frame = self.p_frame.plot(pen=pg.mkPen("#1976d2", width=1.2))
        self.c_frame.setDownsampling(auto=True, method="peak")  # drawing only; zoom shows every sample
        self.c_frame.setClipToView(True)
        self.s_peaks = pg.ScatterPlotItem(size=9, brush=pg.mkBrush("#c62828"), symbol="t")
        self.p_frame.addItem(self.s_peaks)
        self.guide = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen("#ef6c00", width=1.5, style=QtCore.Qt.PenStyle.DashLine))
        self.p_frame.addItem(self.guide)

        glwf = pg.GraphicsLayoutWidget()
        glwf.ci.setSpacing(2)
        self.p_wf = glwf.addPlot(row=0, col=0, title="Waterfall (newest at the top)")
        self.p_wf.setLabel("left", "frame")
        self.p_wf.setLabel("bottom", "distance (mm)")
        self.p_wf.invertY(False)
        self.img = pg.ImageItem(axisOrder="row-major")
        self.img.setAutoDownsample(True)
        cmap = pg.colormap.get("viridis")
        self.img.setColorMap(cmap)
        self.p_wf.addItem(self.img)
        self.cbar = pg.ColorBarItem(values=(0, 1000), colorMap=cmap, interactive=False, width=12)
        self.cbar.setImageItem(self.img, insert_in=self.p_wf)
        self.marker = pg.InfiniteLine(angle=90, movable=True, pen=pg.mkPen("#ef6c00", width=2.5),
                                      hoverPen=pg.mkPen("#ff9800", width=4))
        self.p_wf.addItem(self.marker)

        srow = QtWidgets.QHBoxLayout()
        srow.addWidget(QtWidgets.QLabel("Distance"))
        self.slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.slider.setRange(200, 12000)
        srow.addWidget(self.slider, 1)
        self.sel_spin = DSpin()
        self.sel_spin.setRange(0, 13000)
        self.sel_spin.setDecimals(1)
        self.sel_spin.setSuffix(" mm")
        self.sel_spin.setKeyboardTracking(False)
        srow.addWidget(self.sel_spin)
        self.sel_info = QtWidgets.QLabel("")
        self.sel_info.setMinimumWidth(130)
        self.sel_info.setStyleSheet("color:#ef6c00;")
        srow.addWidget(self.sel_info)
        lay.addLayout(srow)
        # Second row: frame history + amplitude max (same settings as in Display settings, kept in sync)
        srow = QtWidgets.QHBoxLayout()
        srow.addStretch(1)
        srow.addWidget(QtWidgets.QLabel("History"))
        self.hist_frames = ISpin()
        self.hist_frames.setRange(5, LIVE_RING)
        self.hist_frames.setSingleStep(10)
        self.hist_frames.setSuffix(" frames")
        self.hist_frames.setKeyboardTracking(False)
        self.hist_frames.setToolTip("Frames shown in the waterfall and the amplitude history")
        srow.addWidget(self.hist_frames)
        self.hist_secs = QtWidgets.QLabel("")
        self.hist_secs.setMinimumWidth(60)
        srow.addWidget(self.hist_secs)
        srow.addSpacing(12)
        srow.addWidget(QtWidgets.QLabel("Amplitude max"))
        self.amp_row = self.amp_box()
        srow.addWidget(self.amp_row)
        self.amp_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.amp_slider.setRange(0, AMP_SLIDER_MAX // AMP_SLIDER_STEP)   # 0 = auto, steps of 100
        self.amp_slider.setValue(DEFAULT_AMP_MAX // AMP_SLIDER_STEP)
        self.amp_slider.setMinimumWidth(160)
        self.amp_slider.setToolTip(f"Amplitude max, 0-{AMP_SLIDER_MAX} in steps of {AMP_SLIDER_STEP} (far left = auto)")
        self.amp_slider.valueChanged.connect(lambda v: self.on_amp_max(v * AMP_SLIDER_STEP))
        srow.addWidget(self.amp_slider)
        lay.addLayout(srow)

        glh = pg.GraphicsLayoutWidget()
        bottom = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)   # waterfall | history, resizable
        bottom.addWidget(glwf)
        bottom.addWidget(glh)
        bottom.setStretchFactor(0, 1)
        bottom.setStretchFactor(1, 1)
        bottom.setChildrenCollapsible(False)
        lay.addWidget(bottom, 5)
        self.p_hist = glh.addPlot(title="Amplitude history")
        self.p_hist.setLabel("left", "amplitude")
        self.p_hist.setLabel("bottom", "frame")
        self.c_hist = self.p_hist.plot(pen=pg.mkPen("#2e7d32", width=1.2), symbol="o", symbolSize=3,
                                       symbolBrush="#2e7d32", symbolPen=None)

        # zoom / pan
        for p in (self.p_frame, self.p_wf):
            p.setMouseEnabled(x=True, y=False)
            p.getViewBox().setMouseMode(pg.ViewBox.PanMode)
            p.disableAutoRange(axis="x")
        # Same distance range in both plots, by value. (setXLink aligns by screen position, which shows a
        # different range when the plots have different widths, e.g. the waterfall beside the history.)
        self.x_syncing = False
        self.p_frame.sigXRangeChanged.connect(lambda *_: self.sync_x(self.p_frame, self.p_wf))
        self.p_wf.sigXRangeChanged.connect(lambda *_: self.sync_x(self.p_wf, self.p_frame))
        self.p_frame.disableAutoRange(axis="y")      # y is set in steps by draw() (no per-frame rescale)
        self.p_hist.getViewBox().setMouseMode(pg.ViewBox.PanMode)
        for p in (self.p_frame, self.p_wf, self.p_hist):
            p.setMenuEnabled(False)
            p.hideButtons()

        # Sensible axes before the first frame (instead of the library's empty-plot placeholder).
        self.p_frame.setXRange(200, 12000, padding=0.01)
        self.p_frame.setYRange(0, DEFAULT_AMP_MAX, padding=0)
        self.p_wf.setYRange(0, DEFAULT_WATERFALL)
        self.p_hist.setXRange(0, DEFAULT_WATERFALL)
        self.p_hist.setYRange(0, DEFAULT_AMP_MAX, padding=0)

        self.marker.sigPositionChanged.connect(self.on_marker)
        self.marker.sigPositionChangeFinished.connect(lambda: self.select_mm(self.marker.value()))
        self.slider.valueChanged.connect(lambda v: None if self.syncing else self.select_mm(v))
        self.sel_spin.valueChanged.connect(lambda v: None if self.syncing else self.select_mm(v))

        # recording / replay
        r1 = QtWidgets.QHBoxLayout()
        r1.addWidget(QtWidgets.QLabel("Recording"))
        self.rec_sel = Combo()
        r1.addWidget(self.rec_sel, 1)
        b = QtWidgets.QPushButton("Replay")
        b.clicked.connect(self.start_replay)
        r1.addWidget(b)
        lay.addLayout(r1)
        r2 = QtWidgets.QHBoxLayout()
        self.btn_play = QtWidgets.QPushButton("Play")
        self.btn_play.setCheckable(True)
        self.btn_play.toggled.connect(self.toggle_play)
        r2.addWidget(self.btn_play)
        self.pos = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.pos.setRange(0, 0)
        self.pos.valueChanged.connect(self.seek)
        r2.addWidget(self.pos, 1)
        self.pos_label = QtWidgets.QLabel("Frame 0 / 0")
        self.pos_label.setMinimumWidth(110)
        r2.addWidget(self.pos_label)
        self.speed = Combo()
        self.speed.addItems(["0.25x", "0.5x", "1x", "2x", "4x"])
        self.speed.setCurrentIndex(2)
        r2.addWidget(self.speed)
        lay.addLayout(r2)
        self.hist_frames.setValue(DEFAULT_WATERFALL)
        self.hist_frames.valueChanged.connect(self.on_history_frames)
        return w

    # ------------------------------------------------------------ connection / commands
    def toggle_connect(self):
        if self.connected:
            self.request.emit("close", "")
            return
        self.pi_ip = self.ip.text().strip()
        self.sb_conn.setText(f"Connecting to {self.ip.text().strip()} ...")
        self.request.emit("connect", self.ip.text().strip())

    def command(self, line):
        """Send a command without waiting; the reply arrives in on_reply."""
        if not self.connected:
            self.sb_conn.setText(f"'{line}' not sent: not connected")
            return
        self.inflight += 1
        self.request.emit("cmd", line)

    def on_reply(self, cmd, reply):
        if cmd == "connect":
            self.connected = True
            self.btn_connect.setText("Disconnect")
            self.command("get")
            return
        if cmd == "close":
            self.connected = False
            self.btn_connect.setText("Connect")
            self.sb_conn.setText("Disconnected")
            return
        self.inflight = max(0, self.inflight - 1)
        head, self.active, self.pending, self.info = parse_reply(reply)
        self.show_values()
        if head.startswith("err") and cmd != "get":
            QtWidgets.QMessageBox.warning(self, "distance_exe", head)
        self.sb_conn.setText(f"Connected to {self.ip.text().strip()}  |  RSS {self.info.get('rss', '?')}  |  "
                             f"{'running' if self.active.get('running') else 'stopped'}"
                             f"{'' if self.active.get('sensor_ok', 1) else '  |  SENSOR NOT ANSWERING'}"
                             f"{'' if cmd == 'get' else '  |  last: ' + head}")

    def on_failed(self, cmd, err):
        self.inflight = max(0, self.inflight - 1)
        if cmd == "connect":
            self.sb_conn.setText(f"Connect failed: {err}  (is distance_exe running on the Pi?)")
            return
        self.connected = False
        self.btn_connect.setText("Connect")
        self.sb_conn.setText(f"Connection lost ('{cmd}': {err})")

    def poll_settings(self):
        if self.connected and self.inflight == 0:   # keep the view in sync; never queue up polls
            self.command("get")
        self.update_history_secs()
        self.buf_count.setText(str(len(self.rx.recording) if self.rx.recording is not None else
                                   (len(self.recordings[-1].frames) if self.recordings else 0)))
        if getattr(self.rx, "recording_done", None) is not None:  # stopped by the receiver (cap / config)
            self.finish_recording(self.rx.recording_done, self.rx.record_stopped_reason)
            self.rx.recording_done = None

    def apply_preset(self, index):
        """Set every radar setting of the chosen preset, then apply (commands run in order on the Pi)."""
        if index <= 0:
            return
        if not self.connected:
            self.sb_conn.setText("Preset not applied: not connected")
            self.preset.setCurrentIndex(0)
            return
        values = dict(PRESET_BASE, **PRESETS[index - 1][1])
        for key, v in values.items():
            self.command(f"set {key} {v:g}" if isinstance(v, float) else f"set {key} {v}")
        self.command("apply")
        self.sb_conn.setText(f"Applying {PRESETS[index - 1][0].split(':')[0]} ...")

    def send_set(self, key, value):
        if self.syncing:
            return
        if self.widgets.get(key, (None, None, False))[2] and self.preset.currentIndex() != 0:
            self.preset.setCurrentIndex(0)   # a radar field edited by hand: no longer a preset
        v = f"{value:g}" if isinstance(value, float) else str(value)
        self.command(f"set {key} {v}")

    def show_values(self):
        self.syncing = True
        staged = False
        for key, (w, kind, radar) in self.widgets.items():
            if key not in self.pending:
                continue
            v = self.pending[key]
            if not w.hasFocus():
                if kind == "spin":
                    w.setValue(v if isinstance(w, QtWidgets.QDoubleSpinBox) else int(round(v)))
                elif kind == "combo":
                    vals = w.property("values")
                    if int(round(v)) in vals:
                        w.setCurrentIndex(vals.index(int(round(v))))
                else:
                    w.setChecked(bool(v))
            diff = radar and abs(v - self.active.get(key, v)) > 1e-6
            staged |= diff
            css = "background:#fff59d;" if diff else ""
            if w.styleSheet() != css:
                w.setStyleSheet(css)
        self.gain_slider.setValue(int(round(self.pending.get("gain", 0.5) * 100)))
        self.syncing = False
        self.pending_label.setText("Pending changes - press Apply" if staged else "")

    # ------------------------------------------------------------ recording / replay
    def toggle_record(self, on):
        if on:
            if self.mode != "live":
                self.go_live()
            with self.rx.lock:
                self.rx.recording = []
                self.rx.record_cap = self.max_buf.value()
            self.btn_rec.setText("Stop recording")
        else:
            with self.rx.lock:
                frames, self.rx.recording = self.rx.recording, None
            self.finish_recording(frames, "")

    def finish_recording(self, frames, reason):
        self.btn_rec.blockSignals(True)
        self.btn_rec.setChecked(False)
        self.btn_rec.setText("Record")
        self.btn_rec.blockSignals(False)
        if frames:
            rec = Recording(f"Recording {len(self.recordings) + 1} {time.strftime('%H:%M:%S')}", frames)
            self.add_recording(rec)
            if reason:
                self.sb_conn.setText(f"Recording stopped: {reason} ({len(frames)} frames)")

    def add_recording(self, rec):
        self.recordings.append(rec)
        self.rec_sel.addItem(rec.label())
        self.rec_sel.setCurrentIndex(len(self.recordings) - 1)
        self.buf_count.setText(str(len(rec.frames)))

    def save_file(self):
        i = self.rec_sel.currentIndex()
        if i < 0:
            QtWidgets.QMessageBox.information(self, "Save", "Record something first (Record / Stop recording).")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save recording", "a111_distance_recording.npz",
                                                        "Radar recording (*.npz)")
        if path:
            self.recordings[i].save(path, dict(active=self.active, info=self.info))
            self.sb_conn.setText(f"Saved {len(self.recordings[i].frames)} frames to {path}")

    def load_file(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Load recording", "", "Radar recording (*.npz)")
        if path:
            try:
                self.add_recording(Recording.load(path, self.mti_tc.value()))
            except Exception as e:  # file is user input: report, don't crash
                QtWidgets.QMessageBox.warning(self, "Load", f"Could not load {path}: {e}")

    def start_replay(self):
        i = self.rec_sel.currentIndex()
        if i < 0:
            return
        self.replay_rec = self.recordings[i]
        self.mode = "replay"
        n = len(self.replay_rec.frames)
        self.pos.blockSignals(True)
        self.pos.setRange(0, n - 1)
        self.pos.setValue(min(self.wf_frames.value(), n) - 1)
        self.pos.blockSignals(False)
        self.seek(self.pos.value())

    def go_live(self):
        self.btn_play.setChecked(False)
        self.mode = "live"
        self.replay_rec = None
        self.drawn_version = -1

    def toggle_play(self, on):
        self.btn_play.setText("Pause" if on else "Play")
        if on and self.mode == "replay":
            if self.pos.value() >= self.pos.maximum():
                self.pos.setValue(0)
            self.play_step()
        else:
            self.play_timer.stop()

    def play_step(self):
        if not self.btn_play.isChecked() or self.mode != "replay":
            return
        i = self.pos.value()
        fr = self.replay_rec.frames
        if i >= len(fr) - 1:
            self.btn_play.setChecked(False)
            return
        speed = float(self.speed.currentText().rstrip("x"))
        dt = max(fr[i + 1]["t"] - fr[i]["t"], 0.005) / speed
        self.pos.setValue(i + 1)
        self.play_timer.start(int(dt * 1000))

    def seek(self, i):
        if self.mode != "replay" or self.replay_rec is None:
            return
        fr = self.replay_rec.frames
        frames = fr[max(0, i - self.wf_frames.value() + 1): i + 1]
        self.pos_label.setText(f"Frame {i + 1} / {len(fr)}")
        self.draw(frames, Layout(fr[0]["n"], fr[0]["start"], fr[0]["step"]), 0.0)

    def on_history_frames(self, n):
        """One setting, two controls (sidebar + next to the slider): keep both equal, then redraw."""
        for box in (self.wf_frames, getattr(self, "hist_frames", None)):
            if box is not None and box.value() != n:
                box.blockSignals(True)
                box.setValue(n)
                box.blockSignals(False)
        self.update_history_secs()
        self.force_redraw()

    def update_history_secs(self):
        rate = self.rx.radar_rate() if self.mode == "live" else 0.0
        if rate <= 0 and self.mode == "replay" and self.replay_rec and len(self.replay_rec.frames) > 1:
            fr = self.replay_rec.frames
            rate = (len(fr) - 1) / max(fr[-1]["t"] - fr[0]["t"], 1e-6)
        n = self.wf_frames.value()
        text = f"≈ {n / rate:.1f} s" if rate > 0 else ""
        if self.hist_secs.text() != text:
            self.hist_secs.setText(text)

    @staticmethod
    def nice_ticks(lo, hi, target):
        span = hi - lo
        if span <= 0:
            return np.array([])
        raw = span / target
        mag = 10 ** np.floor(np.log10(raw))
        step = mag * min((1, 2, 5, 10), key=lambda k: abs(k * mag - raw))
        return np.arange(np.ceil(lo / step) * step, hi + step * 0.001, step)

    def update_grid(self, *_):
        (x0, x1), (y0, y1) = self.p_frame.viewRange()
        xs, ys = self.nice_ticks(x0, x1, 8), self.nice_ticks(y0, y1, 5)
        gx = np.concatenate([np.repeat(xs, 2), np.tile([x0, x1], len(ys))])
        gy = np.concatenate([np.tile([y0, y1], len(xs)), np.repeat(ys, 2)])
        self.grid.setData(gx, gy)

    def amp_box(self):
        b = ISpin()
        b.setRange(0, 65535)
        b.setSingleStep(100)
        b.setSpecialValueText("auto")
        b.setKeyboardTracking(False)
        b.setValue(DEFAULT_AMP_MAX)
        b.setToolTip("Fixed top of the current-frame amplitude axis; lowest value = auto")
        b.valueChanged.connect(self.on_amp_max)
        return b

    def sync_x(self, src, dst):
        if self.x_syncing:
            return
        self.x_syncing = True
        x0, x1 = src.viewRange()[0]
        dst.setXRange(x0, x1, padding=0)
        self.x_syncing = False

    def on_amp_max(self, v):
        """One setting, two controls (sidebar + next to the slider): keep both equal, then apply."""
        for box in (getattr(self, "amp_side", None), getattr(self, "amp_row", None)):
            if box is not None and box.value() != v:
                box.blockSignals(True)
                box.setValue(v)
                box.blockSignals(False)
        sl = getattr(self, "amp_slider", None)
        if sl is not None:
            pos = min(int(round(v / AMP_SLIDER_STEP)), sl.maximum())
            if sl.value() != pos:
                sl.blockSignals(True)
                sl.setValue(pos)
                sl.blockSignals(False)
        self.ymax = self.ytop = None
        if v > 0:
            self.ytop = float(v)
            self.p_frame.setYRange(0, v, padding=0)
            self.p_hist.setYRange(0, v, padding=0)    # history: same amplitude scale
            self.img.setLevels((0.0, float(v)))      # waterfall colours: same scale, at once (also when paused)
            self.cbar_levels = (0.0, float(v))
            self.cbar.setLevels(self.cbar_levels)
        else:
            self.p_hist.enableAutoRange(axis="y")

    def amp_max(self):
        return self.amp_side.value() if hasattr(self, "amp_side") else DEFAULT_AMP_MAX

    def force_redraw(self):
        if self.mode == "replay":
            self.seek(self.pos.value())
        else:
            self.drawn_version = -1

    # ------------------------------------------------------------ selection
    def select_mm(self, mm):
        if self.layout is None:
            self.selected_mm = float(mm)
            return
        i = self.layout.nearest(mm)
        d = float(self.layout.x_mm[i])
        self.syncing = True
        self.marker.setValue(d)
        self.slider.setValue(int(round(d)))
        self.sel_spin.setValue(d)
        self.syncing = False
        self.show_sample(i)

    def on_marker(self):
        if not self.syncing and self.layout is not None:
            i = self.layout.nearest(self.marker.value())
            if i != self.sel_idx:
                self.syncing = True
                self.slider.setValue(int(round(self.layout.x_mm[i])))
                self.sel_spin.setValue(float(self.layout.x_mm[i]))
                self.syncing = False
                self.show_sample(i)

    def show_sample(self, i):
        self.sel_idx = i
        self.selected_mm = float(self.layout.x_mm[i])
        self.guide.setValue(self.selected_mm)
        self.draw_history()

    def draw_history(self):
        if self.disp is None:
            return
        frames, _, layout = self.disp
        i = min(self.sel_idx, layout.n - 1)
        hx = np.fromiter((f["frame"] for f in frames), np.float64, len(frames))
        hy = np.fromiter((self.env_of(f)[i] for f in frames), np.float32, len(frames))
        ok = np.isfinite(hy)                         # frames without a residual: left out
        self.c_hist.setData(hx[ok], hy[ok])
        info = f"sample {i + 1}/{layout.n}"
        if self.sel_info.text() != info:
            self.sel_info.setText(info)
        title = f"{self.VIEW_TEXT[self.view()][3]} at {layout.x_mm[i]:.1f} mm ({len(frames)} frames)"
        if title != getattr(self, "hist_title", None):   # re-rendering an unchanged title costs a repaint
            self.hist_title = title
            self.p_hist.setTitle(title)

    # ------------------------------------------------------------ drawing
    def reset_view(self):
        if self.layout is not None:
            self.p_frame.setXRange(self.layout.start_mm, self.layout.end_mm, padding=0.01)
        self.ymax = self.ytop = None                  # amplitude axis: fixed value or re-fit on the next frame
        self.p_hist.enableAutoRange(axis="x")
        if self.amp_max() > 0:
            self.p_hist.setYRange(0, self.amp_max(), padding=0)
        else:
            self.p_hist.enableAutoRange(axis="y")

    def setup_layout(self, layout):
        self.layout = layout
        self.syncing = True
        self.slider.setRange(int(np.ceil(layout.start_mm)), int(np.floor(layout.end_mm)))
        self.syncing = False
        self.marker.setBounds((layout.start_mm, layout.end_mm))
        for p in (self.p_frame, self.p_wf):
            p.getViewBox().setLimits(xMin=layout.start_mm - 0.02 * (layout.end_mm - layout.start_mm),
                                     xMax=layout.end_mm + 0.02 * (layout.end_mm - layout.start_mm),
                                     minXRange=20 * layout.step_mm)
        if self.view_layout_key != layout.key:   # keep the user's zoom for the same configuration
            self.view_layout_key = layout.key
            self.reset_view()
        self.select_mm(self.selected_mm)

    def view(self):
        return self.mti_method if self.mti_on else "raw"

    def env_of(self, f):
        """Data shown in the plots: raw envelope, exponential-background MTI, or frame-to-frame Envelope change.
        A frame without a residual (first frame after a reset) gives NaN = nothing drawn."""
        v = self.view()
        if v == "raw":
            return f["env"]
        m = f.get("mti" if v == "exp" else "mti_ff")
        if m is None:
            return np.full(f["n"], np.nan, np.float32)
        return m

    VIEW_TEXT = {"raw": ("Current frame", "amplitude", "Waterfall (newest at the top)", "Amplitude history"),
                 "exp": ("Current frame - MTI exponential background: |envelope - background|",
                         "|envelope - background|", "Waterfall - |envelope - background|", "|envelope - background|"),
                 "ff": ("Current frame - MTI frame-to-frame: Envelope change", "Envelope change",
                        "Waterfall - Envelope change", "Envelope change")}

    def apply_view(self, old):
        v = self.view()
        title, ylab, wft, _ = self.VIEW_TEXT[v]
        self.p_frame.setTitle(title)
        self.p_frame.setLabel("left", ylab)
        self.p_hist.setLabel("left", ylab)
        self.p_wf.setTitle(wft)
        self.hist_title = None
        self.s_peaks.setVisible(v == "raw")          # radar peaks are from the raw signal
        self.on_amp_max(self.amp_max())              # same fixed amplitude max (default 3000) in every view
        self.level = None
        self.redraw_now()

    def toggle_mti(self, on):
        old = self.view()
        self.mti_on = on
        self.btn_mti.setText("MTI: on" if on else "MTI: off")
        self.btn_mti.setStyleSheet("background:#c8e6c9;font-weight:700;" if on else "")
        self.apply_view(old)

    def set_mti_method(self, method):
        old = self.view()
        self.mti_method = method
        if old != self.view():
            self.apply_view(old)

    def on_mti_tc(self, v):
        self.rx.mti.tc = v
        self.rx.mti.reset()

    def redraw_now(self):
        if self.disp is not None:
            frames, _, layout = self.disp
            self.draw(frames, layout, self.rx.radar_rate())

    def display_row(self, f):
        """Waterfall row for drawing: all samples, or (frames > MAX_IMG_COLS samples) the maximum over groups of
        neighbouring samples, so peaks stay visible. Computed once per frame; stored data is not changed."""
        env = self.env_of(f)
        if env.shape[0] <= MAX_IMG_COLS:
            return env
        key = "_disp_" + self.view()
        if env is not f.get("mti") and env is not f.get("mti_ff") and env is not f["env"]:
            key = None                               # NaN placeholder: not cached
        row = f.get(key) if key else None
        if row is None:                     # stored in the frame itself (not saved to recordings)
            k = -(-env.shape[0] // MAX_IMG_COLS)
            row = env[: env.shape[0] // k * k].reshape(-1, k).max(axis=1)
            if key:
                f[key] = row
        return row

    def draw(self, frames, layout, rate):
        if not frames or layout is None:
            return
        if self.layout is None or self.layout.key != layout.key:
            self.disp = None
            self.setup_layout(layout)
        f = frames[-1]
        self.disp = (frames, None, layout)
        env = self.env_of(f)
        self.c_frame.setData(layout.x_mm, env.astype(np.float32))
        self.s_peaks.setData([d * 1000.0 for d, a in f["peaks"]], [a for d, a in f["peaks"]])
        top = float(np.nanpercentile(env, 99.5)) if np.isfinite(env).any() else (self.level or 1.0)
        self.level = top if self.level is None else 0.9 * self.level + 0.1 * top
        q = max(50.0, 10 ** np.floor(np.log10(max(self.level, 1.0))) / 2)   # nice step: 50, 500, ...
        lv = (0.0, max(q, np.ceil(self.level / q) * q))                     # changes only in steps
        if self.amp_max() > 0:
            lv = (0.0, float(self.amp_max()))                                 # same fixed scale as the current frame
        img = np.stack([self.display_row(x) for x in frames])
        self.img.setImage(img, autoLevels=False, levels=lv)
        if lv != getattr(self, "cbar_levels", None):
            self.cbar_levels = lv
            self.cbar.setLevels(lv)
        # Amplitude axis of the current frame: steady (rises at once, falls slowly, in nice steps),
        # so the axis is not re-rendered on every frame. Uses the visible (zoomed) part only.
        fixed = self.amp_max()
        if fixed > 0:
            if self.ytop != fixed:
                self.ytop = float(fixed)
                self.p_frame.setYRange(0, fixed, padding=0)
        x0, x1 = self.p_frame.viewRange()[0]
        i0, i1 = np.searchsorted(layout.x_mm, [x0, x1])
        vis = env[i0:max(i1, i0 + 1)]
        peak = float(np.nanmax(vis)) if vis.size and np.isfinite(vis).any() else (self.ymax or 1.0) / 1.1
        self.ymax = peak * 1.1 if getattr(self, "ymax", None) is None else max(peak * 1.1, self.ymax * 0.97)
        ytop = max(q, np.ceil(self.ymax / q) * q)
        if fixed <= 0 and ytop != getattr(self, "ytop", None):
            self.ytop = ytop
            self.p_frame.setYRange(0, ytop, padding=0)
        y0 = frames[0]["frame"] - 0.5
        half = layout.step_mm / 2.0
        self.img.setRect(QtCore.QRectF(layout.start_mm - half, y0, layout.end_mm - layout.start_mm + 2 * half,
                                       frames[-1]["frame"] - frames[0]["frame"] + 1))
        self.p_wf.setYRange(y0, frames[-1]["frame"] + 0.5, padding=0)
        self.draw_history()
        self.redraws.append(time.monotonic())
        r = self.redraws
        ui = (len(r) - 1) / (r[-1] - r[0]) if len(r) > 2 and r[-1] > r[0] else 0.0
        self.update_stats(f, frames, layout, rate, ui)

    def update_stats(self, f, frames, layout, rate, ui):
        """Two-line stats block above the current frame (same idea as the old trigger window's stats line)."""
        if self.mode == "replay" and len(frames) > 1:       # recordings: rate from the recorded timestamps
            span = frames[-1]["t"] - frames[0]["t"]
            measured = (len(frames) - 1) / span if span > 0 else 0.0
            interval = 1000.0 * span / (len(frames) - 1) if len(frames) > 1 else 0.0
            set_txt, warn, gn_txt = "recorded", "", "-"
        else:
            measured = f["rate"] if f["rate"] > 0 else rate
            interval = f["interval"]
            set_rate = self.active.get("rate", 0.0)
            set_txt = f"set {set_rate:g}" if set_rate > 0 else "set max"
            warn = ("<b style='color:#c62828'>RATE NOT REACHED</b>"
                    if set_rate > 0 and measured < 0.9 * set_rate else "")
            gn_txt = f"{f['get_next']:.2f} ms"
        flags = " ".join(n for b, n in ((F_SAT, "SATURATED"), (F_MISSED, "MISSED DATA"),
                                        (F_QUALITY, "QUALITY WARNING")) if f["flags"] & b)
        peaks = ", ".join(f"{d * 1000:.0f}" for d, a in f["peaks"]) or "none"
        sp = "&nbsp;" * 3
        red = lambda t: f"<b style='color:#c62828'>{t}</b>"
        line1 = f"frame {f['frame']}{sp}measured {measured:.1f} Hz ({set_txt}){sp}{warn + sp if warn else ''}{self.ui_delay_text()}"
        lost = self.rx.pc_lost if self.mode == "live" else 0
        line2 = (f"interval {interval:.2f} ms{sp}get_next {gn_txt}{sp}UI {ui:.1f} Hz{sp}"
                 f"missed frames {f['missed']}{sp}restarts {f['restarts']}"
                 f"{sp + red(f'lost on PC {lost}') if lost else ''}")
        line3 = (f"points {layout.n} x {layout.step_mm:.3f} mm{sp}range {layout.start_mm:.0f}-{layout.end_mm:.0f} mm"
                 f"{sp}peaks (mm) {peaks}{sp + red(flags) if flags else ''}")
        if self.view() == "ff":                      # frame-to-frame: show the reference resets
            if self.mode == "live":
                n_res = sum(self.rx.ff_resets.values())
                line3 = (f"points {layout.n} x {layout.step_mm:.3f} mm{sp}F2F resets {n_res}"
                         f"{' (last: ' + self.rx.ff_last_reason + ')' if self.rx.ff_last_reason else ''}"
                         f"{sp}late/duplicate packets ignored {self.rx.dropped}")
            if f.get("mti_ff") is None:
                line3 += sp + red("no Envelope change for this frame (first frame after a reset)")
        line4 = self.delay_line(f)
        self.info_line.setText(line1 + "<br>" + line2 + "<br>" + line3 + "<br>" + line4)

    def ping_loop(self):
        """Network delay: ping the Pi every 2 s (system ping, 1 packet); half the round-trip ~ one way."""
        while True:
            try:
                out = subprocess.run(["ping", "-c", "1", "-W", "1", self.pi_ip],
                                     capture_output=True, text=True, timeout=3).stdout
                mt = re.search(r"time=([0-9.]+)", out)
                self.net_rtt = float(mt.group(1)) if mt else None
            except (OSError, subprocess.SubprocessError, RuntimeError):
                self.net_rtt = None
            time.sleep(2.0)

    def delay_line(self, f):
        """Line 4 of the stats: radar, network, display, total, and UI input delay."""
        sp = "&nbsp;" * 3
        if self.mode == "replay":
            return "delays: replay (no live radar)"
        radar = f.get("get_next", 0.0)
        net = self.net_rtt / 2.0 if self.net_rtt is not None else None
        disp = getattr(self, "disp_delay", None)
        parts = [f"radar {radar:.1f} ms",
                 f"network &asymp; {net:.0f} ms" if net is not None else "network ?",
                 f"display {disp:.0f} ms" if disp is not None else "display ?"]
        total = radar + (net or 0.0) + (disp or 0.0)
        return f"delays: {sp.join(parts)}{sp}&rarr; total &asymp; {total:.0f} ms"

    def ui_delay_text(self):
        return (f"UI input delay {getattr(self, 'ui_delay_avg', 0.0):.0f} ms "
                f"(max {getattr(self, 'ui_delay_max', 0.0):.0f}, busy {100 * getattr(self, 'busy_share', 0.0):.0f}%)")

    def adapt_ui_rate(self, now):
        """Adaptive redraw rate. The 16 ms timer fires late by exactly the time the window was busy (drawing,
        Qt painting, other work). Every 0.5 s: busy share > 60 % -> redraw less often; < 30 % -> more often.
        Keeps about half of the time free for mouse/keyboard, on any PC and window size."""
        last = getattr(self, "last_tick", None)
        self.last_tick = now
        if last is None:
            self.busy_sum, self.busy_t0 = 0.0, now
            return
        lag = max(0.0, (now - last) - 0.016)
        self.busy_sum += lag
        self.lag_max_w = max(getattr(self, "lag_max_w", 0.0), lag)
        self.lag_n_w = getattr(self, "lag_n_w", 0) + 1
        span = now - self.busy_t0
        if span >= 0.5:
            busy = self.busy_sum / span
            if busy > 0.6:
                self.ui_fps = max(3.0, self.ui_fps * 0.75)
            elif busy < 0.3:
                self.ui_fps = min(float(MAX_UI_FPS), self.ui_fps * 1.25)
            self.busy_share = busy
            self.ui_delay_avg = 1000.0 * self.busy_sum / max(self.lag_n_w, 1)   # ms an input waits on average
            self.ui_delay_max = 1000.0 * self.lag_max_w
            self.lag_max_w, self.lag_n_w = 0.0, 0
            self.busy_sum, self.busy_t0 = 0.0, now

    def refresh(self):
        version, age = self.rx.peek()
        now = time.monotonic()
        self.adapt_ui_rate(now)
        pend = getattr(self, "disp_pending", None)
        if pend is not None:              # previous frame has now been painted
            d = (now - pend) * 1000.0
            self.disp_delay = d if getattr(self, "disp_delay", None) is None else 0.8 * self.disp_delay + 0.2 * d
            self.disp_pending = None
        if version != getattr(self, "seen_version", -1):
            self.seen_version, self.seen_time = version, now
        fresh = version > 0 and now - getattr(self, "seen_time", 0.0) < 1.5
        recording = self.rx.recording is not None
        if self.mode == "replay":
            n = len(self.replay_rec.frames) if self.replay_rec else 0
            self.set_state(f"▶ REPLAY  {self.replay_rec.name if self.replay_rec else ''}", "#6a1b9a")
            return
        if recording:
            self.set_state(f"● RECORDING  {len(self.rx.recording)} frames", "#c62828")
        elif fresh:
            self.set_state("● LIVE", "#2e7d32")
        elif self.connected and not self.active.get("running", 1):
            self.set_state("■ STOPPED", "#757575")
        elif self.connected:
            self.set_state("● CONNECTED, no frames", "#ef6c00")
        else:
            self.set_state("● DISCONNECTED", "#757575")
        if version == self.drawn_version or version == 0:
            return
        now = time.monotonic()
        # redraw rate = adaptive cap, and drawing may use at most half of the time (room for mouse/keyboard)
        if now - getattr(self, "last_draw", 0.0) < 1.0 / self.ui_fps - 0.004:   # tolerance: timer ticks every ~16 ms
            return                                   # newer frames are drawn on a following tick
        self.last_draw = now
        self.drawn_version = version
        frames, layout, rate, _ = self.rx.snapshot(self.wf_frames.value())
        if frames and "rx_t" in frames[-1]:
            self.disp_pending = frames[-1]["rx_t"]
        self.draw(frames, layout, rate)


    def closeEvent(self, e):  # stop the control thread cleanly
        self.ctl_thread.quit()
        self.ctl_thread.wait(2000)
        super().closeEvent(e)

    def set_state(self, text, color):
        if self.state.text() != text:
            self.state.setText(text)
            self.state.setStyleSheet(f"font-weight:700;color:{color};padding:0 8px;")


def main():
    sys.setswitchinterval(0.001)   # let the receiving thread run every 1 ms (default 5 ms) -> no lost frames while painting
    rx = Receiver(UDP_PORT)
    rx.recording_done = None
    rx.start()
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    pal = QtGui.QPalette()
    for role, color in ((QtGui.QPalette.ColorRole.Window, "#f3f3f3"), (QtGui.QPalette.ColorRole.Base, "#ffffff"),
                        (QtGui.QPalette.ColorRole.WindowText, "#212121"), (QtGui.QPalette.ColorRole.Text, "#212121"),
                        (QtGui.QPalette.ColorRole.Button, "#e6e6e6"), (QtGui.QPalette.ColorRole.ButtonText, "#212121")):
        pal.setColor(role, QtGui.QColor(color))
    app.setPalette(pal)
    font = app.font()
    font.setPointSizeF(9.0)
    app.setFont(font)
    pg.setConfigOptions(antialias=False, background="w", foreground="#212121")
    w = Window(rx)
    geo = app.primaryScreen().availableGeometry()
    w.resize(int(geo.width() * 0.5), int(geo.height() * 0.92))
    w.move(geo.x() + geo.width() // 4, geo.y())
    w.show()
    QtCore.QTimer.singleShot(0, lambda: w.split.setSizes([int(w.width() * 0.3), int(w.width() * 0.7)]))
    QtCore.QTimer.singleShot(300, w.toggle_connect)
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
