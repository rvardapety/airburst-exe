#!/usr/bin/env python3
"""Control panel and live view for trigger_exe (src/motion_trigger.c).

- Start / Stop measuring.
- Live settings (threshold, re-arm, cool-down, zone, LED, distance method, ...): applied instantly by
  trigger_exe to every frame, no gap in the data.
- Radar settings (range, rate, profile, HWAAS, gain, sweeps, filters, ...): the Acconeer API cannot change
  them while the sensor measures, so they are staged (yellow) and take effect with Apply
  (deactivate -> reconfigure -> activate). An invalid set is rejected and trigger_exe restores the last good one.
- Plots: motion score with both thresholds and trigger markers, distance (library and interpolated) with the
  zone, and the per-point motion profile of the latest frame.

Everything shown comes from the radar frames trigger_exe sends; nothing is simulated.

Run on the PC:
    ~/acconeer-a111/bin/python tools/trigger_view.py [pi_ip] [udp_port] [tcp_port]
"""
import collections
import socket
import struct
import sys
import threading
import time

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

PI_IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.124"
UDP_PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 5006
TCP_PORT = int(sys.argv[3]) if len(sys.argv) > 3 else 5007
HISTORY_S = 10.0

MAGIC = 0x56573141
HEADER = struct.Struct("<IHHII17f3I")  # 96 bytes, matches motion_trigger.c (version 1)
FLAG_RUNNING, FLAG_LIBDET, FLAG_TRIGGER, FLAG_ARMED, FLAG_LED, FLAG_WARMUP, FLAG_SAT, FLAG_INZONE, \
    FLAG_ZONEON, FLAG_LOG = (1 << i for i in range(10))

# (key, label, kind, min, max, step/decimals or choices, tooltip)
LIVE = [
    ("threshold", "Trigger threshold", "f", 0, 100, 2, "Score above this fires the trigger once"),
    ("rearm", "Re-arm threshold", "f", 0, 100, 2, "Score must drop below this before the next trigger"),
    ("rearm_ms", "Re-arm quiet time (ms)", "f", 0, 60000, 0,
     "Score must stay below the re-arm threshold this long before re-arming (stops repeated triggers)"),
    ("cooldown_ms", "Cool-down (ms)", "f", 0, 60000, 0, "Minimum time after re-arm before the next trigger"),
    ("zone_on", "Zone filter", "b", 0, 1, None, "Trigger only when the distance is inside the zone"),
    ("zone_lo", "Zone from (m)", "f", 0, 15, 3, ""),
    ("zone_hi", "Zone to (m)", "f", 0, 15, 3, ""),
    ("led", "LED output (BCM16)", "b", 0, 1, None, ""),
    ("led_on_ms", "LED on-time (ms)", "f", 0, 60000, 0, ""),
    ("dist_mode", "Distance", "c", 0, 1, ["library (6 cm grid)", "interpolated"], ""),
    ("smooth", "Distance average (frames)", "i", 1, 32, None, ""),
    ("stream_every", "Send every Nth frame", "i", 1, 1000, None, "Viewer load only; detection uses every frame"),
    ("log", "CSV log on the Pi", "b", 0, 1, None, ""),
]
RADAR = [
    ("start", "Range start (m)", "f", 0, 7, 3, "Sparse points are ~6 cm apart"),
    ("length", "Range length (m)", "f", 0.01, 6.7, 3, "Range end (start + length) must be <= 6.72 m"),
    ("rate", "Update rate (Hz)", "f", 0.1, 5000, 1, "Frames per second; host paced, filters depend on it"),
    ("spf", "Sweeps per frame (6-64)", "i", 1, 64, None, "Fewer than 6: no distance points. Points x sweeps must stay <= ~2048"),
    ("sweep_rate", "Sweep rate (Hz, 0 = max)", "f", 0, 100000, 0, ""),
    ("profile", "Profile", "c", 1, 5, ["1 (shortest pulse)", "2", "3", "4", "5 (longest reach)"], ""),
    ("hwaas", "HWAAS (1-63)", "i", 1, 63, None, "Hardware averaging: higher = better SNR, slower"),
    ("gain", "Receiver gain (0-1)", "f", 0, 1, 2, "Lower it if the data saturates"),
    ("downsampling", "Downsampling", "i", 1, 64, None, "Point spacing = 6 cm x this"),
    ("power_save", "Power save", "c", 0, 4, ["off", "sleep", "ready", "active", "hibernate"], ""),
    ("intra_weight", "Intra-frame weight (0-1)", "f", 0, 1, 2, "1 = fast intra-frame score only (quick motion)"),
    ("intra_tc", "Intra-frame time const (s)", "f", 0, 100, 3, ""),
    ("output_tc", "Output time const (s)", "f", 0, 100, 3, "Score smoothing"),
    ("inter_dev_tc", "Inter-frame deviation tc (s)", "f", 0, 100, 3, ""),
    ("inter_fast", "Inter-frame fast cutoff (Hz)", "f", 0, 1000, 2, ""),
    ("inter_slow", "Inter-frame slow cutoff (Hz)", "f", 0, 1000, 2, ""),
    ("pca", "Removed noise components (0-2)", "i", 0, 2, None, "PCA clutter removal"),
    ("async", "Asynchronous measurement", "b", 0, 1, None, ""),
]


def parse(packet):
    if len(packet) < HEADER.size:
        return None
    v = HEADER.unpack_from(packet)
    magic, ver, n, frame, flags = v[:5]
    if magic != MAGIC or ver != 1 or len(packet) < HEADER.size + 4 * n:
        return None
    (t, score, libd, idist, outd, thr, rearm, zlo, zhi, start, step, interval, gn, proc, rate, apply_ms,
     last_trig_d) = v[5:22]
    trig, restarts, missed = v[22:25]
    vec = np.frombuffer(packet, dtype="<f4", count=n, offset=HEADER.size).astype(float)
    return dict(n=n, frame=frame, flags=flags, t=t, score=score, libd=libd, idist=idist, outd=outd, thr=thr,
                rearm=rearm, zlo=zlo, zhi=zhi, start=start, step=step, interval=interval, gn=gn, proc=proc,
                rate=rate, apply_ms=apply_ms, last_trig_d=last_trig_d, triggers=trig, restarts=restarts,
                missed=missed, vec=vec)


class _NoWheel:
    """Settings fields ignore the mouse wheel, so scrolling the panel never changes a value by accident.
    The ignored wheel event goes to the scroll area, which scrolls the panel instead."""

    def wheelEvent(self, event):
        event.ignore()


class NoWheelDoubleSpinBox(_NoWheel, QtWidgets.QDoubleSpinBox):
    pass


class NoWheelSpinBox(_NoWheel, QtWidgets.QSpinBox):
    pass


class NoWheelComboBox(_NoWheel, QtWidgets.QComboBox):
    pass


class Receiver(threading.Thread):
    """Receives every frame. Keeps the history in a fixed numpy ring buffer (cheap to read 20 times a second),
    every trigger in a list (so none is missed between screen updates), and the latest raw packet."""

    SIZE = 16384  # frames kept: > 10 s at up to 1600 Hz

    def __init__(self, port):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.5)
        self.lock = threading.Lock()
        self.buf = np.zeros((self.SIZE, 4))  # t, score, library distance, used distance
        self.head = 0
        self.count = 0
        self.latest_packet = None
        self.triggers = collections.deque(maxlen=500)  # (t, score, distance)
        self.trigger_total = 0
        self.last_rx = 0.0

    def clear(self):
        with self.lock:
            self.head = self.count = 0
            self.triggers.clear()
            self.trigger_total = 0
            self.latest_packet = None
            self.last_rx = 0.0

    def snapshot(self):
        """History in time order (copy), all triggers, the latest frame (parsed) and its age."""
        with self.lock:
            idx = (self.head - self.count + np.arange(self.count)) % self.SIZE
            hist = self.buf[idx]
            trig = list(self.triggers)
            total = self.trigger_total
            packet = self.latest_packet
            age = time.monotonic() - self.last_rx if self.last_rx else None
        return hist, trig, total, (parse(packet) if packet else None), age

    def run(self):
        while True:
            try:
                packet, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            if len(packet) < HEADER.size:
                continue
            v = HEADER.unpack_from(packet)
            if v[0] != MAGIC or v[1] != 1:
                continue
            flags, t, score, libd, outd = v[4], v[5], v[6], v[7], v[9]
            with self.lock:
                if self.count and t < self.buf[(self.head - 1) % self.SIZE, 0]:
                    self.head = self.count = 0  # trigger_exe restarted
                    self.triggers.clear()
                self.buf[self.head] = (t, score, libd, outd)
                self.head = (self.head + 1) % self.SIZE
                self.count = min(self.count + 1, self.SIZE)
                if flags & FLAG_TRIGGER:
                    self.triggers.append((t, score, outd))
                    self.trigger_total += 1
                self.latest_packet = packet
                self.last_rx = time.monotonic()


class Control:
    """TCP command link to trigger_exe: one line out, one line back."""

    def __init__(self):
        self.sock = None
        self.buf = b""

    def connect(self, ip, port):
        self.close()
        self.sock = socket.create_connection((ip, port), timeout=3)
        self.sock.settimeout(5)

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None

    def send(self, line):
        if not self.sock:
            raise OSError("not connected")
        self.sock.sendall((line + "\n").encode())
        while b"\n" not in self.buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise OSError("connection closed")
            self.buf += chunk
        reply, self.buf = self.buf.split(b"\n", 1)
        return reply.decode(errors="replace")


def parse_reply(reply):
    head, _, tail = reply.partition("|")
    active, pending = {}, {}
    for tok in tail.split():
        if "=" not in tok:
            continue
        k, v = tok.split("=", 1)
        if "/" in v:
            a, p = v.split("/", 1)
            active[k], pending[k] = float(a), float(p)
        else:
            active[k] = pending[k] = float(v)
    return head.strip(), active, pending


class Window(QtWidgets.QMainWindow):
    def __init__(self, rx):
        super().__init__()
        self.rx = rx
        self.ctl = Control()
        self.widgets = {}
        self.active = {}
        self.pending = {}
        self.setWindowTitle("Motion trigger - A111 presence detector")
        self.resize(1400, 900)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        outer = QtWidgets.QHBoxLayout(central)

        # ---- left: controls ----
        left = QtWidgets.QWidget()
        lv = QtWidgets.QVBoxLayout(left)
        conn = QtWidgets.QHBoxLayout()
        self.ip_edit = QtWidgets.QLineEdit(PI_IP)
        self.btn_connect = QtWidgets.QPushButton("Connect")
        self.btn_connect.clicked.connect(self.on_connect)
        conn.addWidget(QtWidgets.QLabel("Pi"))
        conn.addWidget(self.ip_edit)
        conn.addWidget(self.btn_connect)
        lv.addLayout(conn)

        ss = QtWidgets.QHBoxLayout()
        self.btn_start = QtWidgets.QPushButton("Start")
        self.btn_stop = QtWidgets.QPushButton("Stop")
        self.btn_restart = QtWidgets.QPushButton("Restart")
        self.btn_restart.setToolTip("From the very beginning: sensor and detector re-initialised, counters and plots cleared")
        for b, cmd, color in ((self.btn_start, "start", "#2e7d32"), (self.btn_stop, "stop", "#c62828"),
                              (self.btn_restart, "restart", "#1565c0")):
            b.setMinimumHeight(40)
            b.setStyleSheet(f"font-size:16px;font-weight:600;color:white;background:{color};")
            b.clicked.connect(lambda _=False, c=cmd: self.command(c))
            ss.addWidget(b)
        lv.addLayout(ss)

        live_box = QtWidgets.QGroupBox("Live settings - instant, no gap")
        live_form = QtWidgets.QFormLayout(live_box)
        for spec in LIVE:
            live_form.addRow(spec[1], self.make_widget(spec, radar=False))
        lv.addWidget(live_box)

        radar_box = QtWidgets.QGroupBox("Radar settings - staged (yellow) until Apply")
        radar_form = QtWidgets.QFormLayout(radar_box)
        for spec in RADAR:
            radar_form.addRow(spec[1], self.make_widget(spec, radar=True))
        btns = QtWidgets.QHBoxLayout()
        for text, cmd in (("Apply", "apply"), ("Revert", "revert"), ("Defaults", "defaults")):
            b = QtWidgets.QPushButton(text)
            b.clicked.connect(lambda _=False, c=cmd: self.command(c))
            btns.addWidget(b)
        radar_form.addRow(btns)
        self.apply_label = QtWidgets.QLabel("")
        radar_form.addRow(self.apply_label)
        lv.addWidget(radar_box)

        self.msg = QtWidgets.QLabel("Not connected")
        self.msg.setWordWrap(True)
        lv.addWidget(self.msg)
        lv.addStretch(1)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(left)
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(430)
        outer.addWidget(scroll, 0)

        # ---- right: readouts + plots ----
        right = QtWidgets.QVBoxLayout()
        self.big = QtWidgets.QLabel("-")
        self.big.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.big.setStyleSheet("font-size:30px;font-weight:700;padding:6px;")
        right.addWidget(self.big)
        self.stats = QtWidgets.QLabel("")
        self.stats.setStyleSheet("font-family:monospace;font-size:13px;")
        right.addWidget(self.stats)
        self.trig_list = QtWidgets.QLabel("Last triggers:  none yet")
        self.trig_list.setStyleSheet("font-family:monospace;font-size:13px;color:#c62828;font-weight:600;")
        right.addWidget(self.trig_list)
        self.seen_triggers = 0
        self.flash_text = ""

        glw = pg.GraphicsLayoutWidget()
        right.addWidget(glw, 1)
        outer.addLayout(right, 1)

        self.p_score = glw.addPlot(row=0, col=0, title="Motion score")
        self.p_score.setLabel("bottom", "seconds ago (0 = now)")
        self.c_score = self.p_score.plot(pen=pg.mkPen("#1976d2", width=2))
        self.l_thr = pg.InfiniteLine(angle=0, pen=pg.mkPen("#c62828", width=1.5), label="trigger",
                                     labelOpts={"position": 0.05})
        self.l_rearm = pg.InfiniteLine(angle=0, pen=pg.mkPen("#ef6c00", width=1.5, style=QtCore.Qt.PenStyle.DashLine),
                                       label="re-arm", labelOpts={"position": 0.15})
        self.p_score.addItem(self.l_thr)
        self.p_score.addItem(self.l_rearm)
        self.s_trig = pg.ScatterPlotItem(size=12, brush=pg.mkBrush("#c62828"), symbol="t")
        self.p_score.addItem(self.s_trig)

        self.p_prof = glw.addPlot(row=1, col=0, title="Motion per distance (latest frame)")
        self.p_prof.setLabel("bottom", "distance", "m")
        self.bars = pg.BarGraphItem(x=[0.0], height=[0.0], width=0.05, brush="#1976d2", pen=pg.mkPen(None))
        self.p_prof.addItem(self.bars)
        self.r_zone_p = pg.LinearRegionItem(movable=False, brush=pg.mkBrush(46, 125, 50, 30))
        self.p_prof.addItem(self.r_zone_p)
        self.l_prof_thr = pg.InfiniteLine(angle=0, pen=pg.mkPen("#c62828", width=1.5), label="trigger",
                                          labelOpts={"position": 0.95, "color": "#c62828"})
        self.p_prof.addItem(self.l_prof_thr)

        # No mouse zoom/pan: the wheel used to zoom a plot and switch its auto-scaling off for good.
        # Axes always follow the data; the right-click menu and the "A" button are removed too.
        for c in (self.c_score,):
            c.setDownsampling(auto=True, method="peak")  # ~1 point per pixel, keeps peaks
            c.setClipToView(True)
        for p in (self.p_score, self.p_prof):
            p.setMouseEnabled(x=False, y=False)
            p.setMenuEnabled(False)
            p.hideButtons()
            p.enableAutoRange()
            for side in ("left", "bottom"):
                p.getAxis(side).enableAutoSIPrefix(False)  # keep plain s and m (no "ks", "mm")

        self.flash_until = 0.0
        self.watchdog = QtCore.QTimer()
        self.watchdog.timeout.connect(self.check_link)
        self.watchdog.start(3000)
        # Settings refresh: shows changes made from elsewhere (e.g. a test script) within a second.
        self.poller = QtCore.QTimer()
        self.poller.timeout.connect(self.poll_settings)
        self.poller.start(1000)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(66)  # 15 screen updates per second

    # ---- settings widgets ----
    def make_widget(self, spec, radar):
        key, _label, kind, lo, hi, extra, tip = spec
        if kind == "f":
            w = NoWheelDoubleSpinBox()
            w.setRange(lo, hi)
            w.setDecimals(extra)
            w.setSingleStep(10 ** -extra if extra else 1)
            w.setKeyboardTracking(False)
            w.valueChanged.connect(lambda v, k=key: self.send_set(k, v))
        elif kind == "i":
            w = NoWheelSpinBox()
            w.setRange(int(lo), int(hi))
            w.setKeyboardTracking(False)
            w.valueChanged.connect(lambda v, k=key: self.send_set(k, v))
        elif kind == "b":
            w = QtWidgets.QCheckBox()
            w.toggled.connect(lambda v, k=key: self.send_set(k, int(v)))
        else:
            w = NoWheelComboBox()
            w.addItems(extra)
            w.currentIndexChanged.connect(lambda i, k=key, base=int(lo): self.send_set(k, i + base))
        w.setToolTip(tip)
        w.setProperty("spec", (kind, int(lo)))
        self.widgets[key] = (w, radar)
        return w

    def show_values(self):
        for key, (w, radar) in self.widgets.items():
            if key not in self.pending:
                continue
            v = self.pending[key]
            kind, lo = w.property("spec")
            if w.hasFocus():
                continue  # don't overwrite a value the user is typing
            w.blockSignals(True)
            if kind in ("f", "i"):
                w.setValue(v if kind == "f" else int(round(v)))
            elif kind == "b":
                w.setChecked(bool(v))
            else:
                w.setCurrentIndex(int(round(v)) - lo)
            w.blockSignals(False)
            staged = radar and abs(self.pending[key] - self.active.get(key, v)) > 1e-6
            w.setStyleSheet("background:#fff59d;" if staged else "")

    # ---- commands ----
    def on_connect(self):
        try:
            self.ctl.connect(self.ip_edit.text().strip(), TCP_PORT)
            self.command("get")
        except OSError as e:
            self.msg.setText(f"Connect failed: {e}  (is trigger_exe running on the Pi?)")

    def send_set(self, key, value):
        self.command(f"set {key} {value:g}" if isinstance(value, float) else f"set {key} {value}")

    def command(self, line):
        if line == "restart":
            self.rx.clear()
            self.seen_triggers = 0
            self.flash_until = 0.0
        t = time.monotonic()
        try:
            reply = self.ctl.send(line)
        except OSError:
            # Link lost (Pi rebooted, Wi-Fi drop, ...): reconnect once and resend.
            try:
                self.ctl.connect(self.ip_edit.text().strip(), TCP_PORT)
                reply = self.ctl.send(line)
            except OSError as e:
                self.msg.setText(f"'{line}' failed: {e}  (is trigger_exe running on the Pi?)")
                self.msg.setStyleSheet("color:#c62828;")
                return
        dt = (time.monotonic() - t) * 1000
        head, self.active, self.pending = parse_reply(reply)
        self.show_values()
        state = "running" if self.active.get("running") else "stopped"
        self.msg.setText(f"{head}   [{state}, sensor {'ok' if self.active.get('sensor_ok') else 'NOT ok'}]")
        self.msg.setStyleSheet("color:#c62828;" if head.startswith("err") else "")
        if line == "restart":
            self.rx.clear()
        if line == "apply":
            self.apply_label.setText(f"Last apply: {self.active.get('apply_ms', 0):.1f} ms on the Pi "
                                     f"({dt:.0f} ms round trip); filters warming up")

    def poll_settings(self):
        if not self.ctl.sock:
            return
        try:
            reply = self.ctl.send("get")
        except OSError:
            self.ctl.close()  # the watchdog reconnects
            return
        _head, self.active, self.pending = parse_reply(reply)
        self.show_values()

    def check_link(self):
        """No frames for 3 s: reconnect, so the Pi streams to this PC again."""
        with self.rx.lock:
            last = self.rx.last_rx
        if last and time.monotonic() - last < 3.0:
            return
        if self.active.get("running", 1) == 0:
            return  # stopped on purpose
        try:
            self.ctl.connect(self.ip_edit.text().strip(), TCP_PORT)
            self.command("get")
        except OSError:
            pass

    # ---- plots ----
    def refresh(self):
        hist, trig, total, f, age = self.rx.snapshot()
        if total < self.seen_triggers:
            self.seen_triggers = 0  # receiver was cleared
        if total > self.seen_triggers:  # a trigger arrived since the last screen update (never missed)
            self.seen_triggers = total
            self.flash_until = time.monotonic() + 1.5
            self.flash_text = f"TRIGGER at {trig[-1][2]:.3f} m   (score {trig[-1][1]:.1f})"
        if f is None or (age is not None and age > 1.0):
            if self.ctl.sock and self.active.get("running", 1) == 0:
                self.big.setText("STOPPED - press Start")
            elif self.ctl.sock and self.active.get("sensor_ok", 1) == 0:
                self.big.setText("Sensor not answering - retrying (check the XR112)")
            elif self.ctl.sock:
                self.big.setText(f"Connected, waiting for frames on UDP {UDP_PORT}")
            else:
                self.big.setText("Not connected - press Connect")
            self.big.setStyleSheet("font-size:30px;font-weight:700;padding:6px;background:#e0e0e0;color:#333;")
            if f is None or len(hist) == 0:
                return
        t_end = hist[-1, 0]
        a = hist[hist[:, 0] >= t_end - HISTORY_S]
        x = a[:, 0] - t_end  # time relative to the newest frame: -10 ... 0 s
        self.c_score.setData(x, a[:, 1])
        tr = [x for x in trig if x[0] >= t_end - HISTORY_S]
        self.s_trig.setData([x[0] - t_end for x in tr], [x[1] for x in tr])
        recent = trig[-6:][::-1]
        self.trig_list.setText("Last triggers:  " + ("   |   ".join(
            f"{t_end - t:5.1f} s ago: {d:.3f} m (score {sc:.1f})" for t, sc, d in recent) if recent else "none yet"))
        self.l_thr.setValue(f["thr"])
        self.l_rearm.setValue(f["rearm"])
        zone_on = bool(f["flags"] & FLAG_ZONEON)
        self.r_zone_p.setVisible(zone_on)
        self.r_zone_p.setRegion((f["zlo"], f["zhi"]))
        if f["n"]:
            x = f["start"] + np.arange(f["n"]) * f["step"]
            self.bars.setOpts(x=x, height=f["vec"], width=f["step"] * 0.8)
        self.l_prof_thr.setValue(f["thr"])

        stale = age is not None and age > 1.0
        if stale:
            pass  # status text already set above
        elif not (f["flags"] & FLAG_RUNNING):
            self.big.setText("STOPPED - press Start")
            self.big.setStyleSheet("font-size:30px;font-weight:700;padding:6px;background:#e0e0e0;color:#333;")
        elif time.monotonic() < self.flash_until:
            self.big.setText(self.flash_text)
            self.big.setStyleSheet("font-size:30px;font-weight:700;padding:6px;background:#c62828;color:white;")
        elif f["flags"] & FLAG_WARMUP:
            self.big.setText("warming up filters")
            self.big.setStyleSheet("font-size:30px;font-weight:700;padding:6px;background:#fff59d;color:#333;")
        else:
            state = "armed" if f["flags"] & FLAG_ARMED else "waiting for score < re-arm"
            if f["score"] >= f["rearm"]:
                self.big.setText(f"{state}   motion at {f['outd']:.3f} m   score {f['score']:.2f}")
            else:
                self.big.setText(f"{state} - no motion   (score {f['score']:.2f})")
            self.big.setStyleSheet("font-size:30px;font-weight:700;padding:6px;")

        req = self.active.get("rate", 0)
        warn = "  RATE NOT REACHED" if req and f["rate"] and f["rate"] < 0.9 * req else ""
        self.stats.setText(
            f"frame {f['frame']}  measured {f['rate']:.1f} Hz (set {req:g}){warn}   interval {f['interval']:.2f} ms   "
            f"get_next {f['gn']:.2f} ms   frame-to-LED {f['proc']:.3f} ms\n"
            f"triggers {f['triggers']}   missed frames {f['missed']}   restarts {f['restarts']}   "
            f"points {f['n']} x {f['step'] * 100:.0f} cm   library distance {f['libd']:.3f} m   "
            f"interpolated {f['idist']:.3f} m   "
            f"{'SATURATED  ' if f['flags'] & FLAG_SAT else ''}{'LED ON  ' if f['flags'] & FLAG_LED else ''}"
            f"{'logging' if f['flags'] & FLAG_LOG else ''}")


def main():
    rx = Receiver(UDP_PORT)
    rx.start()
    app = QtWidgets.QApplication(sys.argv)
    # Light mode regardless of the desktop theme: light Qt palette + white plot background.
    app.setStyle("Fusion")
    pal = QtGui.QPalette()
    for role, color in ((QtGui.QPalette.ColorRole.Window, "#f5f5f5"), (QtGui.QPalette.ColorRole.Base, "#ffffff"),
                        (QtGui.QPalette.ColorRole.AlternateBase, "#eeeeee"), (QtGui.QPalette.ColorRole.Button, "#e8e8e8"),
                        (QtGui.QPalette.ColorRole.WindowText, "#212121"), (QtGui.QPalette.ColorRole.Text, "#212121"),
                        (QtGui.QPalette.ColorRole.ButtonText, "#212121"), (QtGui.QPalette.ColorRole.ToolTipBase, "#ffffff"),
                        (QtGui.QPalette.ColorRole.ToolTipText, "#212121"), (QtGui.QPalette.ColorRole.Highlight, "#1976d2"),
                        (QtGui.QPalette.ColorRole.HighlightedText, "#ffffff")):
        pal.setColor(role, QtGui.QColor(color))
    app.setPalette(pal)
    pg.setConfigOptions(antialias=False, background="w", foreground="#212121")
    w = Window(rx)
    w.show()
    QtCore.QTimer.singleShot(200, w.on_connect)
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
