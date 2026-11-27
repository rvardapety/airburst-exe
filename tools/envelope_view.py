#!/usr/bin/env python3
"""Live viewer for envelope_exe (src/service_envelope.c).

Receives one UDP packet per radar frame from the Raspberry Pi and plots:
  - the measured envelope (reflected amplitude vs distance),
  - the empty-scene background and the detection threshold,
  - the detection region around 0.30 m (shaded),
  - peaks: local maxima where the measurement is above the threshold,
  - the detection status, frame rate and last detection time.

Everything drawn comes from the received measurements; nothing is simulated.

Run on the PC (uses PySide6 + pyqtgraph, already in ~/acconeer-a111):
    ~/acconeer-a111/bin/python tools/envelope_view.py [port] [log.csv]

With log.csv given, every received packet is logged with its PC arrival time
(time.monotonic, taken in a receiver thread the moment the packet arrives).
"""
import socket
import struct
import sys
import threading
import time

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 5005
LOG_PATH = sys.argv[2] if len(sys.argv) > 2 else None
MAGIC = 0x56453141
HEADER = struct.Struct("<IHHIfffffff4BIddfffff4Bf")  # 92 bytes, matches service_envelope.c (version 3)
DIRECTIONS = {0: "direction unknown", 1: "approaching", 2: "receding", 3: "not moving"}


def parse(packet, with_data=True):
    if len(packet) < HEADER.size:
        return None
    (magic, version, n, frame, start, step, lo, hi, fps, peak_m, peak_excess,
     detected, calibrated, saturated, missed, detections, t_s, last_t,
     get_next_ms, proc_ms, band_over, gpeak_m, gpeak_amp, above, event, direction, _r2,
     velocity) = HEADER.unpack_from(packet)
    if magic != MAGIC or version != 3 or len(packet) < HEADER.size + 6 * n:
        return None
    f = dict(n=n, frame=frame, start=start, step=step, lo=lo, hi=hi, fps=fps, peak_m=peak_m,
             peak_excess=peak_excess, detected=bool(detected), calibrated=bool(calibrated),
             saturated=bool(saturated), missed=bool(missed), detections=detections, t_s=t_s,
             last_t=last_t, get_next_ms=get_next_ms, proc_ms=proc_ms, band_over=band_over,
             gpeak_m=gpeak_m, gpeak_amp=gpeak_amp, above=bool(above), event=bool(event),
             direction=direction, velocity=velocity)
    if with_data:
        data = np.frombuffer(packet, dtype="<u2", count=3 * n, offset=HEADER.size).astype(float)
        f.update(env=data[:n], bg=data[n:2 * n], thr=data[2 * n:])
    return f


class Receiver(threading.Thread):
    """Receives every packet, timestamps it on arrival, logs it, keeps the latest for the UI."""

    def __init__(self, port, log_path):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.5)
        self.lock = threading.Lock()
        self.latest = None
        self.running = True
        self.log = open(log_path, "w") if log_path else None
        if self.log:
            self.log.write("frame,t_pi_s,t_pc_s,get_next_ms,proc_ms,missed,saturated,calibrated,above,"
                           "detected,event,band_over,peak_m,gpeak_m,gpeak_amp,velocity_mps,direction\n")

    def run(self):
        last_flush = time.monotonic()
        while self.running:
            try:
                packet, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                if self.log:
                    self.log.flush()  # write the tail of a run once packets stop
                continue
            t_pc = time.monotonic()
            with self.lock:
                self.latest = packet
            if self.log:
                f = parse(packet, with_data=False)
                if f is not None:
                    self.log.write("%d,%.6f,%.6f,%.3f,%.3f,%d,%d,%d,%d,%d,%d,%.1f,%.4f,%.4f,%d,%.3f,%s\n" % (
                        f["frame"], f["t_s"], t_pc, f["get_next_ms"], f["proc_ms"], f["missed"],
                        f["saturated"], f["calibrated"], f["above"], f["detected"], f["event"],
                        f["band_over"], f["peak_m"], f["gpeak_m"], int(f["gpeak_amp"]), f["velocity"],
                        DIRECTIONS.get(f["direction"], "?")))
                if t_pc - last_flush > 1.0:
                    self.log.flush()
                    last_flush = t_pc
        if self.log:
            self.log.close()

    def take_latest(self):
        with self.lock:
            packet, self.latest = self.latest, None
        return packet


class Viewer(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("A111 Envelope - object at 0.30 m")
        self.resize(1000, 600)

        self.receiver = Receiver(PORT, LOG_PATH)
        self.receiver.start()

        layout = QtWidgets.QVBoxLayout(self)
        self.status = QtWidgets.QLabel("Waiting for radar data on UDP port %d ..." % PORT)
        self.status.setAlignment(QtCore.Qt.AlignCenter)
        self.status.setStyleSheet("font-size: 28px; font-weight: bold; padding: 8px;")
        layout.addWidget(self.status)
        self.info = QtWidgets.QLabel("")
        self.info.setStyleSheet("font-size: 14px; padding: 2px;")
        layout.addWidget(self.info)

        self.plot = pg.PlotWidget()
        self.plot.setBackground("w")
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setLabel("bottom", "Distance from radar (m)")
        self.plot.getAxis("bottom").enableAutoSIPrefix(False)
        self.plot.setLabel("left", "Reflected amplitude (envelope)")
        self.plot.addLegend(offset=(10, 10))
        layout.addWidget(self.plot)

        self.region = pg.LinearRegionItem(values=(0.25, 0.35), movable=False,
                                          brush=pg.mkBrush(255, 200, 0, 50), pen=pg.mkPen(None))
        self.plot.addItem(self.region)
        self.bg_curve = self.plot.plot(name="Background (empty scene)",
                                       pen=pg.mkPen((120, 120, 120), width=1, style=QtCore.Qt.DashLine))
        self.thr_curve = self.plot.plot(name="Detection threshold", pen=pg.mkPen((220, 120, 0), width=1.5))
        self.env_curve = self.plot.plot(name="Measured envelope", pen=pg.mkPen((30, 90, 200), width=2))
        self.peaks = pg.ScatterPlotItem(size=11, brush=pg.mkBrush(220, 30, 30), pen=pg.mkPen("k"),
                                        name="Peaks above threshold")
        self.plot.addItem(self.peaks)

        self.ymax = 0.0
        self.seen_detections = 0
        self.last_distance = None
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_view)
        self.timer.start(30)

    def closeEvent(self, event):
        self.receiver.running = False
        self.receiver.join(timeout=1.0)
        super().closeEvent(event)

    def update_view(self):
        packet = self.receiver.take_latest()
        if packet is None:
            return
        f = parse(packet)
        if f is None:
            return

        x = f["start"] + np.arange(f["n"]) * f["step"]
        self.env_curve.setData(x, f["env"])
        self.region.setRegion((f["lo"], f["hi"]))

        if f["calibrated"]:
            self.bg_curve.setData(x, f["bg"])
            self.thr_curve.setData(x, f["thr"])
            env, thr = f["env"], f["thr"]
            local_max = np.zeros_like(env, dtype=bool)
            local_max[1:-1] = (env[1:-1] >= env[:-2]) & (env[1:-1] > env[2:])
            idx = np.where(local_max & (env > thr))[0]
            self.peaks.setData(x[idx], env[idx])
        else:
            self.bg_curve.setData([], [])
            self.thr_curve.setData([], [])
            self.peaks.setData([], [])

        self.ymax = max(self.ymax * 0.97,float(f["env"].max()) * 1.15, float(f["thr"].max()) * 1.1, 200.0)
        self.plot.setYRange(0, self.ymax, padding=0)
        self.plot.setXRange(x[0], x[-1], padding=0.01)

        if not f["calibrated"]:
            self.status.setText("Calibrating background - keep the area clear")
            self.status.setStyleSheet("font-size: 28px; font-weight: bold; padding: 8px; color: #806000;")
        elif f["detected"]:
            motion = DIRECTIONS.get(f["direction"], "direction unknown")
            if f["direction"] in (1, 2):
                motion += " %.2f m/s" % abs(f["velocity"])
            where = " at %.3f m" % f["peak_m"] if f["peak_m"] > 0 else ""
            self.status.setText("Object detected%s  -  %s" % (where, motion))
            self.status.setStyleSheet("font-size: 28px; font-weight: bold; padding: 8px; "
                                      "color: white; background: #c62828;")
        else:
            self.status.setText("No object in %.2f-%.2f m" % (f["lo"], f["hi"]))
            self.status.setStyleSheet("font-size: 28px; font-weight: bold; padding: 8px; color: #2e7d32;")

        if f["detections"] != self.seen_detections:
            # a new detection was confirmed in this or a recent frame: remember its distance
            self.seen_detections = f["detections"]
            self.last_distance = f["peak_m"] if f["peak_m"] > 0 else None
        last = "%.2f s" % f["last_t"] if f["last_t"] >= 0 else "-"
        if self.last_distance is not None and f["last_t"] >= 0:
            last += " at %.3f m" % self.last_distance
        flags = (" | SATURATED" if f["saturated"] else "") + (" | MISSED FRAME" if f["missed"] else "")
        speed = ("radial speed %+.2f m/s (%s)" % (f["velocity"], DIRECTIONS.get(f["direction"], "?"))
                 if f["direction"] != 0 else "radial speed -")
        self.info.setText("Frame %d  |  %.1f Hz measured  |  t = %.2f s  |  detections: %d  |  last detection at %s  |  %s%s"
                          % (f["frame"], f["fps"], f["t_s"], f["detections"], last, speed, flags))


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    viewer = Viewer()
    viewer.show()
    sys.exit(app.exec())
