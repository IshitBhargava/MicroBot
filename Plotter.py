#!/usr/bin/env python3
"""Light serial plotter. Input lines: title:val,title:val,...

pip install pyqt5 matplotlib pyserial
"""
import sys
import time
from collections import deque

import serial
import serial.tools.list_ports
from PyQt5 import QtCore, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as Canvas
from matplotlib.figure import Figure

MAXPTS = 20000
BAUDS = ["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600", "2000000"]


class Plotter(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Serial Plotter")
        self.ser = None
        self.buf = b""
        self.t0 = time.perf_counter()
        self.data = {}    # title -> (deque t, deque v)
        self.items = {}   # title -> QListWidgetItem
        self.lines = {}   # title -> Line2D
        self.dirty = False
        self.relegend = True
        self.paused = False

        # --- controls ---
        self.port = QtWidgets.QComboBox()
        self.port.setMinimumWidth(120)
        self.baud = QtWidgets.QComboBox()
        self.baud.setEditable(True)
        self.baud.addItems(BAUDS)
        self.baud.setCurrentText("115200")
        self.btn_refresh = QtWidgets.QPushButton("Refresh")
        self.btn_conn = QtWidgets.QPushButton("Connect")
        self.btn_pause = QtWidgets.QPushButton("Pause")
        self.btn_pause.setCheckable(True)
        self.btn_clear = QtWidgets.QPushButton("Clear")
        self.win = QtWidgets.QDoubleSpinBox()
        self.win.setRange(0, 3600)
        self.win.setValue(10)
        self.win.setSuffix(" s")
        self.win.setToolTip("Time window (0 = show all)")

        top = QtWidgets.QHBoxLayout()
        for w in (QtWidgets.QLabel("Port"), self.port, self.btn_refresh,
                  QtWidgets.QLabel("Baud"), self.baud, self.btn_conn,
                  QtWidgets.QLabel("Window"), self.win, self.btn_pause, self.btn_clear):
            top.addWidget(w)
        top.addStretch()

        # --- channel list ---
        self.list = QtWidgets.QListWidget()
        self.list.setMaximumWidth(180)
        side = QtWidgets.QVBoxLayout()
        side.addWidget(QtWidgets.QLabel("Channels"))
        side.addWidget(self.list)
        row = QtWidgets.QHBoxLayout()
        b_all = QtWidgets.QPushButton("All")
        b_none = QtWidgets.QPushButton("None")
        row.addWidget(b_all)
        row.addWidget(b_none)
        side.addLayout(row)

        # --- plot ---
        self.fig = Figure(tight_layout=True)
        self.ax = self.fig.add_subplot(111)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel("t (s)")
        self.canvas = Canvas(self.fig)

        body = QtWidgets.QHBoxLayout()
        body.addLayout(side)
        body.addWidget(self.canvas, 1)
        root = QtWidgets.QVBoxLayout()
        root.addLayout(top)
        root.addLayout(body, 1)
        c = QtWidgets.QWidget()
        c.setLayout(root)
        self.setCentralWidget(c)
        self.resize(1000, 600)

        # --- signals ---
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_conn.clicked.connect(self.toggle_conn)
        self.btn_pause.toggled.connect(lambda v: setattr(self, "paused", v))
        self.btn_clear.clicked.connect(self.clear)
        self.list.itemChanged.connect(self.on_check)
        b_all.clicked.connect(lambda: self.set_all(QtCore.Qt.Checked))
        b_none.clicked.connect(lambda: self.set_all(QtCore.Qt.Unchecked))

        # --- timers ---
        self.rt = QtCore.QTimer(self)
        self.rt.timeout.connect(self.read)
        self.rt.start(10)
        self.dt = QtCore.QTimer(self)
        self.dt.timeout.connect(self.draw)
        self.dt.start(50)

        self.refresh_ports()

    # ---------- serial ----------
    def refresh_ports(self):
        cur = self.port.currentText()
        self.port.clear()
        self.port.addItems([p.device for p in serial.tools.list_ports.comports()])
        if cur:
            self.port.setCurrentText(cur)

    def toggle_conn(self):
        if self.ser:
            self.disconnect()
            return
        try:
            self.ser = serial.Serial(self.port.currentText(), int(self.baud.currentText()), timeout=0)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "Serial error", str(e))
            self.ser = None
            return
        self.buf = b""
        self.btn_conn.setText("Disconnect")
        self.port.setEnabled(False)
        self.baud.setEnabled(False)

    def disconnect(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.btn_conn.setText("Connect")
        self.port.setEnabled(True)
        self.baud.setEnabled(True)

    def read(self):
        if not self.ser:
            return
        try:
            n = self.ser.in_waiting
            if not n:
                return
            self.buf += self.ser.read(n)
        except Exception as e:
            self.disconnect()
            QtWidgets.QMessageBox.warning(self, "Serial error", str(e))
            return
        *lines, self.buf = self.buf.split(b"\n")
        if len(self.buf) > 4096:  # no newline in sight, drop garbage
            self.buf = b""
        if self.paused:
            return
        t = time.perf_counter() - self.t0
        for ln in lines:
            self.parse(ln.decode(errors="ignore").strip(), t)

    def parse(self, line, t):
        for part in line.split(","):
            k, sep, v = part.partition(":")
            if not sep:
                continue
            k = k.strip()
            try:
                val = float(v)
            except ValueError:
                continue
            if k not in self.data:
                self.add_channel(k)
            ts, vs = self.data[k]
            ts.append(t)
            vs.append(val)
            self.dirty = True

    # ---------- channels ----------
    def add_channel(self, k):
        self.data[k] = (deque(maxlen=MAXPTS), deque(maxlen=MAXPTS))
        it = QtWidgets.QListWidgetItem(k)
        it.setFlags(it.flags() | QtCore.Qt.ItemIsUserCheckable)
        self.list.blockSignals(True)
        it.setCheckState(QtCore.Qt.Checked)
        self.list.addItem(it)
        self.list.blockSignals(False)
        self.items[k] = it
        self.relegend = True

    def on_check(self, _):
        self.relegend = True
        self.dirty = True

    def set_all(self, state):
        self.list.blockSignals(True)
        for it in self.items.values():
            it.setCheckState(state)
        self.list.blockSignals(False)
        self.on_check(None)

    def clear(self):
        self.t0 = time.perf_counter()
        for ts, vs in self.data.values():
            ts.clear()
            vs.clear()
        self.dirty = True

    # ---------- drawing ----------
    def draw(self):
        if not self.dirty:
            return
        self.dirty = False
        tmax = 0.0
        for k, it in self.items.items():
            on = it.checkState() == QtCore.Qt.Checked
            ln = self.lines.get(k)
            if ln is None:
                ln, = self.ax.plot([], [], label=k, lw=1)
                self.lines[k] = ln
            ln.set_visible(on)
            if on:
                ts, vs = self.data[k]
                ln.set_data(ts, vs)
                if ts:
                    tmax = max(tmax, ts[-1])
        if self.relegend:
            self.relegend = False
            vis = [l for l in self.lines.values() if l.get_visible()]
            if vis:
                self.ax.legend(handles=vis, loc="upper left", fontsize=8)
            elif self.ax.get_legend():
                self.ax.get_legend().remove()
        w = self.win.value()
        self.ax.relim(visible_only=True)
        self.ax.autoscale_view(scalex=(w == 0))
        if w > 0:
            self.ax.set_xlim(max(0, tmax - w), max(w, tmax))
            self.autoscale_y_in_window(tmax - w, tmax)
        self.canvas.draw_idle()

    def autoscale_y_in_window(self, lo, hi):
        ys = []
        for k, ln in self.lines.items():
            if not ln.get_visible():
                continue
            ts, vs = self.data[k]
            ys.extend(v for t, v in zip(ts, vs) if t >= lo)
        if ys:
            a, b = min(ys), max(ys)
            pad = (b - a) * 0.05 or 1
            self.ax.set_ylim(a - pad, b + pad)

    def closeEvent(self, e):
        self.disconnect()
        e.accept()


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = Plotter()
    w.show()
    sys.exit(app.exec_())
