#!/usr/bin/env python3
"""
IMU pyqtgraph visualizer (5s rolling window)
- Plots accel (m/s^2), gyro (deg/s), mag (uT)
- Optional VQF roll/pitch/yaw (deg) if `vqf` is installed
- Input: serial (--port) or file (--file)

Install:
  pip install pyqtgraph PyQt5 pyserial vqf

Examples:
  python imu_printfpa.py --port /dev/ttyACM0 --baud 115200 --gyro-units rad --rpy-3d
  python imu_printfpa.py --file sample.csv --gyro-units rad
  python imu_printfpa.py --file sample.csv --no-orientation
    python imu_printfpa.py --file sample.csv --plots accel gyro fpa
"""
import sys, math, time, threading, queue, argparse, re
from collections import deque

import numpy as np

# -------- optional deps ----------
try:
    import serial  # pyserial (only needed for --port)
except Exception:
    serial = None
try:
    from vqf import VQF
    HAVE_VQF = True
except Exception:
    # vqf python module intentionally not used in this GUI; disable
    HAVE_VQF = False


# -------- GUI deps (required) ----
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore

try:
    import pyqtgraph.opengl as gl
    HAVE_GL = True
except Exception:
    gl = None
    HAVE_GL = False


class FootOrientationState:
    def __init__(self):
        self.lock = threading.Lock()
        self.roll = math.nan
        self.pitch = math.nan
        self.yaw = math.nan

    def update(self, roll_deg, pitch_deg, yaw_deg):
        with self.lock:
            self.roll = roll_deg
            self.pitch = pitch_deg
            self.yaw = yaw_deg

    def snapshot(self):
        with self.lock:
            return self.roll, self.pitch, self.yaw


FOOT_ORIENTATION = FootOrientationState()
FOOT_DEBUG_RE = re.compile(
    r"footRoll_deg=([-+]?\d*\.?\d+|nan),"
    r"footPitch_deg=([-+]?\d*\.?\d+|nan),"
    r"footYaw_deg=([-+]?\d*\.?\d+|nan)",
    re.IGNORECASE,
)


def update_foot_orientation_from_serial(line: str):
    m = FOOT_DEBUG_RE.search(line)
    if not m:
        return
    vals = []
    for token in m.groups():
        try:
            vals.append(float(token))
        except ValueError:
            vals.append(float("nan"))
    FOOT_ORIENTATION.update(vals[0], vals[1], vals[2])


class RPY3DWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None, axis_len=0.3):
        super().__init__(parent)
        self.setWindowTitle("RPY 3D View")
        self.resize(520, 520)

        self.axis_len = axis_len
        self.glview = gl.GLViewWidget()
        self.glview.setBackgroundColor((30, 30, 30, 255))
        self.glview.opts['distance'] = 0.8
        self.glview.opts['elevation'] = 18
        self.glview.opts['azimuth'] = 45

        central = QtWidgets.QWidget(self)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.glview)
        self.setCentralWidget(central)

        grid = gl.GLGridItem()
        grid.scale(0.1, 0.1, 0.1)
        self.glview.addItem(grid)

        self.axis_items = []
        colors = [
            (1.0, 0.2, 0.2, 1.0),  # X axis (red)
            (0.2, 1.0, 0.2, 1.0),  # Y axis (green)
            (0.2, 0.4, 1.0, 1.0),  # Z axis (blue)
        ]
        origin = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=float)
        for color in colors:
            item = gl.GLLinePlotItem(pos=origin.copy(), color=color, width=3, antialias=True, mode='lines')
            self.glview.addItem(item)
            self.axis_items.append(item)

        self.set_orientation(0.0, 0.0, 0.0)

    @staticmethod
    def _rpy_to_matrix(roll_deg, pitch_deg, yaw_deg):
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)
        sr, cr = math.sin(roll), math.cos(roll)
        sp, cp = math.sin(pitch), math.cos(pitch)
        sy, cy = math.sin(yaw), math.cos(yaw)
        return np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,                 cp * cr               ],
        ], dtype=float)

    def set_orientation(self, roll_deg, pitch_deg, yaw_deg):
        if not self.axis_items:
            return
        r = 0.0 if not math.isfinite(roll_deg) else roll_deg
        p = 0.0 if not math.isfinite(pitch_deg) else pitch_deg
        y = 0.0 if not math.isfinite(yaw_deg) else yaw_deg
        R = self._rpy_to_matrix(r, p, y)
        for idx in range(3):
            axis_vec = R[:, idx] * self.axis_len
            pts = np.array([[0.0, 0.0, 0.0], axis_vec], dtype=float)
            self.axis_items[idx].setData(pos=pts)


class SensorPlotWindow(QtWidgets.QMainWindow):
    def __init__(self, show_accel, show_gyro, show_mag, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Sensor Streams")
        self.resize(900, 720)

        central = QtWidgets.QWidget(self)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(6, 6, 6, 6)
        self.graph = pg.GraphicsLayoutWidget(show=True)
        layout.addWidget(self.graph)
        self.setCentralWidget(central)

        self.plots = {}
        self.curves = {}
        self._layout_started = False

        def make_plot(title):
            if self._layout_started:
                self.graph.nextRow()
            else:
                self._layout_started = True
            plot = self.graph.addPlot(title=title)
            plot.addLegend()
            plot.showGrid(x=True, y=True, alpha=0.3)
            return plot

        if show_accel:
            plot = make_plot("Acceleration (m/s²)")
            self.plots["accel"] = plot
            self.curves["ax"] = plot.plot([], [], pen=pg.mkPen((255, 120, 120), width=2), name="ax")
            self.curves["ay"] = plot.plot([], [], pen=pg.mkPen((120, 255, 120), width=2), name="ay")
            self.curves["az"] = plot.plot([], [], pen=pg.mkPen((120, 170, 255), width=2), name="az")

        if show_gyro:
            plot = make_plot("Gyroscope (deg/s)")
            self.plots["gyro"] = plot
            self.curves["gx"] = plot.plot([], [], pen=pg.mkPen((255, 180, 120), width=2), name="gx")
            self.curves["gy"] = plot.plot([], [], pen=pg.mkPen((180, 255, 120), width=2), name="gy")
            self.curves["gz"] = plot.plot([], [], pen=pg.mkPen((180, 200, 255), width=2), name="gz")

        if show_mag:
            plot = make_plot("Magnetometer (µT)")
            self.plots["mag"] = plot
            self.curves["mx"] = plot.plot([], [], pen=pg.mkPen((255, 220, 120), width=2), name="mx")
            self.curves["my"] = plot.plot([], [], pen=pg.mkPen((160, 255, 200), width=2), name="my")
            self.curves["mz"] = plot.plot([], [], pen=pg.mkPen((200, 200, 255), width=2), name="mz")


def parse_args():
    ap = argparse.ArgumentParser()
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="Serial port (e.g., /dev/ttyACM0 or COM5)")
    src.add_argument("--file", help="CSV file with IMU lines")
    ap.add_argument("--baud", type=int, default=115200, help="Serial baudrate")
    ap.add_argument("--window", type=float, default=5.0, help="Rolling window (seconds)")
    ap.add_argument("--ui-hz", type=float, default=60.0, help="UI refresh rate")
    ap.add_argument("--gyro-units", choices=["rad","deg"], default="rad",
                    help="Units of gx,gy,gz in the incoming stream")
    ap.add_argument("--no-orientation", action="store_true",
                    help="Disable VQF orientation")
    ap.add_argument("--queue-max", type=int, default=10000, help="Reader→GUI queue size")
    ap.add_argument("--plots", nargs="+",
                    choices=["accel", "gyro", "mag", "orientation", "fpa", "foot-axis"],
                    default=["accel", "gyro", "mag", "orientation", "fpa", "foot-axis"],
                    help="Plots to display (e.g. --plots accel gyro fpa foot-axis). Default shows all;"
                         " foot-axis requires pyqtgraph.opengl.")
    ap.add_argument("--rpy-3d", action="store_true",
                    help="Show extra roll/pitch/yaw window (requires pyqtgraph.opengl)")
    return ap.parse_args()


def parse_line(line):
    """
    Parse a CSV line from the firmware stream.

    Format:
      t_ms,ax,ay,az,gx,gy,gz,mx,my,mz[,FPA_deg=..,stride=..,dx=..,dy=..,Ts=..]

    Returns tuple (t_ms, ax, ay, az, gx, gy, gz, mx, my, mz, extras_dict)
    where extras_dict may be empty when no additional fields are present.
    Returns None if malformed.
    """
    s = line.strip()
    if not s:
        return None
    if s.startswith("# FPA dbg"):
        print(s)
        return None
    if s.startswith("#"):
        return None
    update_foot_orientation_from_serial(s)
    # print(FOOT_ORIENTATION.snapshot())
    parts = s.split(",")
    if len(parts) < 10: return None
    base_fields = parts[:10]
    extras_fields = parts[10:]

    try:
        vals = [float("nan") if p.strip().lower()=="nan" else float(p) for p in base_fields]
    except ValueError:
        return None

    extras = {}
    for token in extras_fields:
        token = token.strip()
        if not token:
            continue
        if "=" not in token:
            continue
        key, raw_val = token.split("=", 1)
        key = key.strip().lower()
        raw_val = raw_val.strip()
        if not key:
            continue
        try:
            if key == "stride":
                extras[key] = int(float(raw_val))
            else:
                extras[key] = float(raw_val)
        except ValueError:
            continue

    t_ms = vals[0]
    return (t_ms, *vals[1:], extras)


class ReaderThread(threading.Thread):
    """Background reader: serial or file → queue of parsed records."""
    def __init__(self, port, baud, filepath, out_q, stop_evt):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.filepath = filepath
        self.q = out_q
        self.stop_evt = stop_evt
        self.ser = None
        self.fp = None
        self.cmd_q = queue.Queue()

    def open(self):
        if self.filepath:
            self.fp = open(self.filepath, "r", encoding="utf-8")
            return True
        if serial is None:
            print("pyserial not installed. Use --file or pip install pyserial", file=sys.stderr)
            return False
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            return True
        except Exception as e:
            print(f"Failed to open serial: {e}", file=sys.stderr)
            return False

    def readline(self):
        if self.fp:
            return self.fp.readline()
        if self.ser:
            b = self.ser.readline()
            if isinstance(b, bytes):
                try:
                    return b.decode("utf-8", "ignore")
                except Exception:
                    return ""
        return ""

    def close(self):
        try:
            if self.fp: self.fp.close()
        except: pass
        try:
            if self.ser and self.ser.is_open: self.ser.close()
        except: pass

    def run(self):
        if not self.open():
            return
        try:
            while not self.stop_evt.is_set():
                line = self.readline()
                if not line:
                    time.sleep(0.001)  # be nice to CPU
                    continue
                rec = parse_line(line)
                if rec is None:
                    continue
                try:
                    self.q.put_nowait(rec)
                except queue.Full:
                    # drop if GUI is behind (keeps latency low)
                    pass
                # send any pending commands
                try:
                    cmd = self.cmd_q.get_nowait()
                    if self.ser and self.ser.is_open:
                        self.ser.write((cmd + '\n').encode('utf-8'))
                except queue.Empty:
                    pass
        finally:
            self.close()


class IMUWindow(QtWidgets.QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.setWindowTitle("IMU Live (pyqtgraph)")
        self.resize(1200, 800)

        self.args = args
        self.win = args.window

        # Determine which plots are active
        all_plots = ["accel", "gyro", "mag", "orientation", "fpa", "foot-axis"]
        self.plots_enabled = set(args.plots or all_plots)
        if "foot-axis" in self.plots_enabled and not HAVE_GL:
            print("[imu_printfpa] foot-axis plot requested but pyqtgraph.opengl not available; disabling.",
                  file=sys.stderr)
            self.plots_enabled.discard("foot-axis")

        # Enable foot-axis if orientation is enabled (since foot-axis shows foot orientation from serial)
        if "orientation" in self.plots_enabled and HAVE_GL:
            self.plots_enabled.add("foot-axis")

        # Orientation plotting will use serial-provided footRoll/pitch/yaw values.
        # If user explicitly disables orientation, honor that flag.
        if args.no_orientation and ("orientation" in self.plots_enabled):
            print(f"[imu_printfpa] orientation plot disabled via --no-orientation", file=sys.stderr)
            self.plots_enabled.discard("orientation")

        self.sensor_window = None
        self.sensor_plots = {}
        self.sensor_curves = {}
        if any(p in self.plots_enabled for p in ("accel", "gyro", "mag")):
            self.sensor_window = SensorPlotWindow(
                show_accel="accel" in self.plots_enabled,
                show_gyro="gyro" in self.plots_enabled,
                show_mag="mag" in self.plots_enabled,
                parent=self,
            )
            self.sensor_window.show()
            self.sensor_plots = dict(self.sensor_window.plots)
            self.sensor_curves = dict(self.sensor_window.curves)
            self.sensor_window.destroyed.connect(self._on_sensor_window_destroyed)

        # Data buffers (deques of (t, value))
        self.t0 = None
        self.t_last = 0.0
        self.buf = {}
        if "accel" in self.plots_enabled:
            self.buf.update({"ax": deque(), "ay": deque(), "az": deque()})
        if "gyro" in self.plots_enabled:
            self.buf.update({"gx": deque(), "gy": deque(), "gz": deque()})
        if "mag" in self.plots_enabled:
            self.buf.update({"mx": deque(), "my": deque(), "mz": deque()})
        if "orientation" in self.plots_enabled:
            self.buf.update({"roll": deque(), "pitch": deque(), "yaw": deque()})
        if "fpa" in self.plots_enabled:
            self.buf["fpa"] = deque()
        self._samples_key = next(iter(self.buf), None)

        # Orientation (from serial footRoll/pitch/yaw)
        # Whether to compute VQF locally (disabled if vqf module not available)
        self.use_vqf = (not args.no_orientation) and HAVE_VQF
        self.vqf = VQF() if self.use_vqf else None
        self.prev_t = None

        # Foot progression angle tracking
        self.fpa_latest = math.nan
        self.fpa_stride = None
        self.fpa_dx = math.nan
        self.fpa_dy = math.nan
        self.fpa_stride_time = math.nan
        self.foot_roll = math.nan
        self.foot_pitch = math.nan
        self.foot_yaw = math.nan

        self.plot_acc = self.cur_ax = self.cur_ay = self.cur_az = None
        self.plot_gyr = self.cur_gx = self.cur_gy = self.cur_gz = None
        self.plot_mag = self.cur_mx = self.cur_my = self.cur_mz = None
        self.plot_rpy = self.cur_roll = self.cur_pitch = self.cur_yaw = None
        self.plot_fpa = self.cur_fpa = None

        self.plot_acc = self.sensor_plots.get("accel")
        self.cur_ax = self.sensor_curves.get("ax")
        self.cur_ay = self.sensor_curves.get("ay")
        self.cur_az = self.sensor_curves.get("az")
        self.plot_gyr = self.sensor_plots.get("gyro")
        self.cur_gx = self.sensor_curves.get("gx")
        self.cur_gy = self.sensor_curves.get("gy")
        self.cur_gz = self.sensor_curves.get("gz")
        self.plot_mag = self.sensor_plots.get("mag")
        self.cur_mx = self.sensor_curves.get("mx")
        self.cur_my = self.sensor_curves.get("my")
        self.cur_mz = self.sensor_curves.get("mz")

        # GL failure message (set when init/draw fails)
        self._gl_failed_msg = None

        self.rpy3d_window = None

        # Numeric RPY readouts (always present so user can verify orientation)
        self.rpy_widget = None
        self.rpy_labels = (None, None, None)

        # UI layout
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        vbox = QtWidgets.QVBoxLayout(central)
        self.glw = pg.GraphicsLayoutWidget(show=True)
        vbox.addWidget(self.glw)

        layout_started = False

        def make_plot(title: str):
            nonlocal layout_started
            if layout_started:
                self.glw.nextRow()
            else:
                layout_started = True
            return self.glw.addPlot(title=title)

        if "orientation" in self.plots_enabled and self.use_vqf:
            self.plot_rpy = make_plot("Orientation (VQF, deg)")
            self.plot_rpy.addLegend()
            self.plot_rpy.showGrid(x=True, y=True, alpha=0.3)
            self.cur_roll  = self.plot_rpy.plot([], [], pen=pg.mkPen((255,140,140), width=2), name="roll")
            self.cur_pitch = self.plot_rpy.plot([], [], pen=pg.mkPen((140,255,140), width=2), name="pitch")
            self.cur_yaw   = self.plot_rpy.plot([], [], pen=pg.mkPen((140,140,255), width=2), name="yaw")

        if "fpa" in self.plots_enabled:
            self.plot_fpa = make_plot("Foot Progression Angle (deg)")
            self.plot_fpa.addLegend()
            self.plot_fpa.showGrid(x=True, y=True, alpha=0.3)
            self.cur_fpa = self.plot_fpa.plot([], [], pen=pg.mkPen((255,255,120), width=2), name="FPA")

        self.glview = None
        self.axis_items = []
        self.axis_len = 0.15
        # Foot-axis: show 3D GL view when available and always provide a 2D XY fallback
        self.fallback_axes_plot = None
        self.axis2d_lines = []
        if "foot-axis" in self.plots_enabled:
            if HAVE_GL:
                self.glview = gl.GLViewWidget()
                self.glview.setBackgroundColor((30, 30, 30, 255))
                self.glview.opts['distance'] = 0.6
                self.glview.opts['elevation'] = 18
                self.glview.opts['azimuth'] = 45
                grid = gl.GLGridItem()
                grid.scale(0.1, 0.1, 0.1)
                self.glview.addItem(grid)
                self._init_foot_axes()
                vbox.addWidget(self.glview, stretch=1)
                # small status label for GL diagnostics
                try:
                    self.gl_status_label = QtWidgets.QLabel("GL: initialized")
                except Exception:
                    self.gl_status_label = None
                if self.gl_status_label is not None:
                    vbox.addWidget(self.gl_status_label, stretch=0)

            # 2D fallback: simple XY projection of the 3 axes (always created so user always sees orientation)
            self.fallback_axes_plot = pg.PlotWidget(title="Foot Axis (XY projection)")
            self.fallback_axes_plot.showGrid(x=True, y=True, alpha=0.3)
            # create three colored lines for X (red), Y (green), Z (blue)
            self.axis2d_lines = [
                self.fallback_axes_plot.plot([], [], pen=pg.mkPen((255, 51, 51), width=3), name="X"),
                self.fallback_axes_plot.plot([], [], pen=pg.mkPen((51, 255, 51), width=3), name="Y"),
                self.fallback_axes_plot.plot([], [], pen=pg.mkPen((51, 102, 255), width=3), name="Z"),
            ]
            self.fallback_axes_plot.setAspectLocked(True)
            # set a visible default range centered at origin
            rng = self.axis_len * 2.0
            self.fallback_axes_plot.setXRange(-rng, rng, padding=0)
            self.fallback_axes_plot.setYRange(-rng, rng, padding=0)
            vbox.addWidget(self.fallback_axes_plot, stretch=0)

        if args.rpy_3d:
            if not HAVE_GL:
                print("[imu_printfpa] --rpy-3d requested but pyqtgraph.opengl unavailable; ignoring.",
                      file=sys.stderr)
            else:
                try:
                    self.rpy3d_window = RPY3DWindow(parent=self)
                    self.rpy3d_window.show()
                    self.rpy3d_window.raise_()
                except Exception as exc:
                    print(f"[imu_printfpa] Failed to initialize RPY 3D window: {exc}", file=sys.stderr)
                    self.rpy3d_window = None

            # Numeric RPY readout: large labels for Roll / Pitch / Yaw
            self.rpy_widget = QtWidgets.QWidget()
            rpy_layout = QtWidgets.QHBoxLayout(self.rpy_widget)
            self.rpy_lbl_roll = QtWidgets.QLabel("Roll: --.-°")
            self.rpy_lbl_pitch = QtWidgets.QLabel("Pitch: --.-°")
            self.rpy_lbl_yaw = QtWidgets.QLabel("Yaw: --.-°")
            for lbl in (self.rpy_lbl_roll, self.rpy_lbl_pitch, self.rpy_lbl_yaw):
                lbl.setStyleSheet("font-size:18px; padding:6px;")
                rpy_layout.addWidget(lbl)
            vbox.addWidget(self.rpy_widget, stretch=0)

            # GL enable/disable checkbox (runtime toggle)
            self.gl_enable_cb = QtWidgets.QCheckBox("Enable 3D GL")
            self.gl_enable_cb.setChecked(HAVE_GL and ("foot-axis" in self.plots_enabled))
            self.gl_enable_cb.stateChanged.connect(self._on_gl_toggle)
            vbox.addWidget(self.gl_enable_cb, stretch=0)
            # If GL already failed during init, propagate the error to label and disable checkbox
            if getattr(self, '_gl_failed_msg', None):
                try:
                    if hasattr(self, 'gl_status_label') and self.gl_status_label is not None:
                        self.gl_status_label.setText(self._gl_failed_msg)
                except Exception:
                    pass
                try:
                    self.gl_enable_cb.setChecked(False)
                except Exception:
                    pass

        # Reader thread + timer to drain queue
        self.q = queue.Queue(maxsize=args.queue_max)
        self.stop_evt = threading.Event()
        self.reader = ReaderThread(args.port, args.baud, args.file, self.q, self.stop_evt)
        self.cmd_q = self.reader.cmd_q
        self.reader.start()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.on_timer)
        self.timer.start(int(1000 / max(1.0, args.ui_hz)))

        # Parameter controls
        self.controls = self.make_controls()
        vbox.addWidget(self.controls)

        # status text
        self.status = self.statusBar()
        self.samples_in_window = 0

    def closeEvent(self, ev):
        self.stop_evt.set()
        try:
            self.reader.join(timeout=1.0)
        except Exception:
            pass
        try:
            if self.rpy3d_window is not None:
                self.rpy3d_window.close()
        except Exception:
            pass
        try:
            if self.sensor_window is not None:
                self.sensor_window.close()
        except Exception:
            pass
        ev.accept()

    def send_command(self, cmd):
        self.cmd_q.put(cmd)

    def make_controls(self):
        group = QtWidgets.QGroupBox("Parameter Controls")
        form = QtWidgets.QFormLayout(group)

        # VQF params
        self.vqf_tau_acc_edit = QtWidgets.QLineEdit("3.0")
        form.addRow("VQF Tau Acc (s):", self.vqf_tau_acc_edit)

        self.vqf_tau_mag_edit = QtWidgets.QLineEdit("9.0")
        form.addRow("VQF Tau Mag (s):", self.vqf_tau_mag_edit)

        self.vqf_motion_bias_cb = QtWidgets.QCheckBox()
        self.vqf_motion_bias_cb.setChecked(True)
        form.addRow("VQF Motion Bias:", self.vqf_motion_bias_cb)

        self.vqf_rest_bias_cb = QtWidgets.QCheckBox()
        self.vqf_rest_bias_cb.setChecked(True)
        form.addRow("VQF Rest Bias:", self.vqf_rest_bias_cb)

        self.vqf_mag_reject_cb = QtWidgets.QCheckBox()
        self.vqf_mag_reject_cb.setChecked(True)
        form.addRow("VQF Mag Reject:", self.vqf_mag_reject_cb)

        self.vqf_use_mag_cb = QtWidgets.QCheckBox()
        self.vqf_use_mag_cb.setChecked(False)
        form.addRow("VQF Use Mag:", self.vqf_use_mag_cb)

        # FPA params
        self.fpa_offset_edit = QtWidgets.QLineEdit("0.0")
        form.addRow("FPA Offset (deg):", self.fpa_offset_edit)

        self.fpa_median_edit = QtWidgets.QLineEdit("1")
        form.addRow("FPA Median Win:", self.fpa_median_edit)

        self.fpa_lowpass_edit = QtWidgets.QLineEdit("0.0")
        form.addRow("FPA Lowpass (Hz):", self.fpa_lowpass_edit)

        self.fpa_acc_diff_edit = QtWidgets.QLineEdit("0.455")
        form.addRow("FPA Acc Diff Th:", self.fpa_acc_diff_edit)

        self.fpa_gyr_norm_edit = QtWidgets.QLineEdit("1.0")
        form.addRow("FPA Gyr Norm Th:", self.fpa_gyr_norm_edit)

        self.fpa_hys_frac_edit = QtWidgets.QLineEdit("0.15")
        form.addRow("FPA Hys Frac:", self.fpa_hys_frac_edit)

        self.fpa_min_rest_edit = QtWidgets.QLineEdit("0.08")
        form.addRow("FPA Min Rest (s):", self.fpa_min_rest_edit)

        self.fpa_min_motion_edit = QtWidgets.QLineEdit("0.06")
        form.addRow("FPA Min Motion (s):", self.fpa_min_motion_edit)

        # Buttons
        hbox = QtWidgets.QHBoxLayout()
        send_vqf_btn = QtWidgets.QPushButton("Send VQF")
        send_vqf_btn.clicked.connect(self.send_vqf_params)
        hbox.addWidget(send_vqf_btn)

        send_fpa_btn = QtWidgets.QPushButton("Send FPA")
        send_fpa_btn.clicked.connect(self.send_fpa_params)
        hbox.addWidget(send_fpa_btn)

        send_all_btn = QtWidgets.QPushButton("Send All")
        send_all_btn.clicked.connect(self.send_all_params)
        hbox.addWidget(send_all_btn)

        form.addRow(hbox)

        return group

    def send_vqf_params(self):
        tau_acc = self.vqf_tau_acc_edit.text()
        self.send_command(f"vqf.tau_acc={tau_acc}")

        tau_mag = self.vqf_tau_mag_edit.text()
        self.send_command(f"vqf.tau_mag={tau_mag}")

        motion_bias = 1 if self.vqf_motion_bias_cb.isChecked() else 0
        self.send_command(f"vqf.motion_bias={motion_bias}")

        rest_bias = 1 if self.vqf_rest_bias_cb.isChecked() else 0
        self.send_command(f"vqf.rest_bias={rest_bias}")

        mag_reject = 1 if self.vqf_mag_reject_cb.isChecked() else 0
        self.send_command(f"vqf.mag_reject={mag_reject}")

        use_mag = 1 if self.vqf_use_mag_cb.isChecked() else 0
        self.send_command(f"vqf.use_mag={use_mag}")

    def send_fpa_params(self):
        offset = self.fpa_offset_edit.text()
        self.send_command(f"fpa.offset_deg={offset}")

        median = self.fpa_median_edit.text()
        self.send_command(f"fpa.median_win={median}")

        lowpass = self.fpa_lowpass_edit.text()
        self.send_command(f"fpa.lowpass_hz={lowpass}")

        acc_diff = self.fpa_acc_diff_edit.text()
        self.send_command(f"fpa.acc_diff_th={acc_diff}")

        gyr_norm = self.fpa_gyr_norm_edit.text()
        self.send_command(f"fpa.gyr_norm_th={gyr_norm}")

        hys_frac = self.fpa_hys_frac_edit.text()
        self.send_command(f"fpa.hys_frac={hys_frac}")

        min_rest = self.fpa_min_rest_edit.text()
        self.send_command(f"fpa.min_rest_s={min_rest}")

        min_motion = self.fpa_min_motion_edit.text()
        self.send_command(f"fpa.min_motion_s={min_motion}")

    def send_all_params(self):
        self.send_vqf_params()
        self.send_fpa_params()

    # ---------- helpers ----------
    @staticmethod
    def quat_to_euler_zyx(w,x,y,z):
        # ZYX: yaw(Z), pitch(Y), roll(X)
        s = max(min(2*(w*y - z*x), 1.0), -1.0)
        yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        pitch = math.asin(s)
        roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        return roll, pitch, yaw

    def append(self, key, t, val):
        d = self.buf.get(key)
        if d is None:
            return
        d.append((t, val))
        # drop older than window
        t_cut = t - self.win
        while d and d[0][0] < t_cut:
            d.popleft()

    def series(self, key):
        d = self.buf.get(key)
        if not d:
            return [], []
        ts = [t for (t, _) in d]
        vs = [v for (_, v) in d]
        return ts, vs

    def pull_latest_foot_orientation(self):
        roll, pitch, yaw = FOOT_ORIENTATION.snapshot()
        self.foot_roll = roll
        self.foot_pitch = pitch
        self.foot_yaw = yaw

    @staticmethod
    def _rpy_to_matrix(roll_deg, pitch_deg, yaw_deg):
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)
        sr, cr = math.sin(roll), math.cos(roll)
        sp, cp = math.sin(pitch), math.cos(pitch)
        sy, cy = math.sin(yaw), math.cos(yaw)
        return np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,                 cp * cr               ]
        ], dtype=float)

    def update_rpy3d_window(self):
        if self.rpy3d_window is None:
            return
        roll = self.foot_roll if math.isfinite(self.foot_roll) else 0.0
        pitch = self.foot_pitch if math.isfinite(self.foot_pitch) else 0.0
        yaw = self.foot_yaw if math.isfinite(self.foot_yaw) else 0.0
        try:
            self.rpy3d_window.set_orientation(roll, pitch, yaw)
        except Exception as exc:
            print(f"[imu_printfpa] Failed to update RPY 3D window: {exc}", file=sys.stderr)

    def _init_foot_axes(self):
        if not HAVE_GL or self.glview is None:
            return
        colors = [
            (1.0, 0.2, 0.2, 1.0),  # X axis (red)
            (0.2, 1.0, 0.2, 1.0),  # Y axis (green)
            (0.2, 0.4, 1.0, 1.0),  # Z axis (blue)
        ]
        origin = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=float)
        for color in colors:
            try:
                # use 'lines' mode which is more compatible in some environments
                item = gl.GLLinePlotItem(pos=origin.copy(), color=color, width=3, antialias=True, mode='lines')
                self.axis_items.append(item)
                self.glview.addItem(item)
            except Exception as e:
                # record error to status label and continue
                if hasattr(self, 'gl_status_label') and self.gl_status_label is not None:
                    self.gl_status_label.setText(f"GL init error: {e}")
        # Try to nudge camera distance so small axes are visible
        try:
            self.glview.opts['distance'] = max(self.glview.opts.get('distance', 0.6), 0.2)
        except Exception:
            pass
        self.update_foot_axes(force_identity=True)

    def _on_gl_toggle(self, state):
        # show/hide the GL widget based on checkbox state
        enabled = bool(state)
        if not HAVE_GL:
            # can't enable
            try:
                self.gl_enable_cb.setChecked(False)
            except Exception:
                pass
            return
        try:
            if enabled and self.glview is None:
                # try to create glview now
                try:
                    self.glview = gl.GLViewWidget()
                    self.glview.setBackgroundColor((30, 30, 30, 255))
                    self._init_foot_axes()
                    # insert before fallback_axes_plot if layout available
                    # best-effort: add to main vbox
                    self.centralWidget().layout().insertWidget(0, self.glview)
                except Exception as e:
                    self._gl_failed_msg = f"GL init error: {e}"
                    if hasattr(self, 'gl_status_label') and self.gl_status_label is not None:
                        self.gl_status_label.setText(self._gl_failed_msg)
                    self.gl_enable_cb.setChecked(False)
            elif not enabled and self.glview is not None:
                try:
                    # remove widget from layout
                    self.centralWidget().layout().removeWidget(self.glview)
                    self.glview.setParent(None)
                    self.glview = None
                except Exception:
                    pass
        except Exception:
            pass

    def update_foot_axes(self, force_identity=False):
        # Update 3D GL axes if present
        if HAVE_GL and self.glview is not None and self.axis_items:
            if force_identity:
                roll = pitch = yaw = 0.0
            else:
                roll = self.foot_roll if math.isfinite(self.foot_roll) else 0.0
                pitch = self.foot_pitch if math.isfinite(self.foot_pitch) else 0.0
                yaw = self.foot_yaw if math.isfinite(self.foot_yaw) else 0.0
            R = self._rpy_to_matrix(roll, pitch, yaw)
            for idx in range(3):
                axis_vec = R[:, idx] * self.axis_len
                pts = np.array([[0.0, 0.0, 0.0], axis_vec], dtype=float)
                try:
                    # GLLinePlotItem expects a (N,3) array for 'pos'
                    self.axis_items[idx].setData(pos=pts)
                except Exception as e:
                    msg = f"GL draw error: {e}"
                    self._gl_failed_msg = msg
                    if hasattr(self, 'gl_status_label') and self.gl_status_label is not None:
                        self.gl_status_label.setText(msg)
                    # hide GL widget and uncheck the checkbox
                    try:
                        if self.glview is not None:
                            self.centralWidget().layout().removeWidget(self.glview)
                            self.glview.setParent(None)
                            self.glview = None
                        if hasattr(self, 'gl_enable_cb'):
                            self.gl_enable_cb.setChecked(False)
                    except Exception:
                        pass

        # Update 2D fallback XY projection if present (always update so user sees orientation)
        if self.fallback_axes_plot is not None and self.axis2d_lines:
            if force_identity:
                roll = pitch = yaw = 0.0
            else:
                roll = self.foot_roll if math.isfinite(self.foot_roll) else 0.0
                pitch = self.foot_pitch if math.isfinite(self.foot_pitch) else 0.0
                yaw = self.foot_yaw if math.isfinite(self.foot_yaw) else 0.0
            R = self._rpy_to_matrix(roll, pitch, yaw)
            # X-axis projection (X vs Y)
            x_axis = R[:, 0] * self.axis_len
            y_axis = R[:, 1] * self.axis_len
            z_axis = R[:, 2] * self.axis_len
            # For 2D view we'll plot X and Y components in XY plane
            self.axis2d_lines[0].setData([0.0, x_axis[0]], [0.0, x_axis[1]])
            self.axis2d_lines[1].setData([0.0, y_axis[0]], [0.0, y_axis[1]])
            self.axis2d_lines[2].setData([0.0, z_axis[0]], [0.0, z_axis[1]])

    def _on_sensor_window_destroyed(self, _obj=None):
        self.sensor_window = None
        self.sensor_plots = {}
        self.sensor_curves = {}
        self.plot_acc = self.cur_ax = self.cur_ay = self.cur_az = None
        self.plot_gyr = self.cur_gx = self.cur_gy = self.cur_gz = None
        self.plot_mag = self.cur_mx = self.cur_my = self.cur_mz = None

    def on_timer(self):
        # drain queue
        drained = 0
        while True:
            try:
                rec = self.q.get_nowait()
            except queue.Empty:
                break
            drained += 1
            extras = {}
            if len(rec) >= 11 and isinstance(rec[-1], dict):
                t_ms, ax, ay, az, gx, gy, gz, mx, my, mz, extras = rec
            else:
                t_ms, ax, ay, az, gx, gy, gz, mx, my, mz = rec[:10]
            t = t_ms * 1e-3
            if self.t0 is None:
                self.t0 = t
            trel = t - self.t0
            self.t_last = trel

            # store accel
            self.append("ax", trel, ax)
            self.append("ay", trel, ay)
            self.append("az", trel, az)

            # store gyro (plot in deg/s regardless of input units)
            if self.args.gyro_units == "rad":
                gxd = math.degrees(gx); gyd = math.degrees(gy); gzd = math.degrees(gz)
                g_for_vqf = (gx, gy, gz)
            else:
                gxd = gx; gyd = gy; gzd = gz
                g_for_vqf = (math.radians(gx), math.radians(gy), math.radians(gz))
            self.append("gx", trel, gxd)
            self.append("gy", trel, gyd)
            self.append("gz", trel, gzd)

            # store mag
            self.append("mx", trel, mx)
            self.append("my", trel, my)
            self.append("mz", trel, mz)

            # orientation (optional)
            if self.use_vqf:
                dt = 0.0 if self.prev_t is None else max(0.0, (t - (self.prev_t)))
                self.prev_t = t
                have_mag = not (math.isnan(mx) or math.isnan(my) or math.isnan(mz))
                if have_mag:
                    self.vqf.update(g_for_vqf, (ax,ay,az), (mx,my,mz), dt=dt)
                    w,x,y,z = self.vqf.getQuat9D()
                else:
                    self.vqf.update_imu(g_for_vqf, (ax,ay,az), dt=dt)
                    w,x,y,z = self.vqf.getQuat6D()
                roll, pitch, yaw = [math.degrees(v) for v in self.quat_to_euler_zyx(w,x,y,z)]
                self.append("roll", trel, roll)
                self.append("pitch", trel, pitch)
                self.append("yaw", trel, yaw)

            if extras:
                fpa_deg = extras.get("fpa_deg")
                if fpa_deg is not None and math.isfinite(fpa_deg):
                    self.append("fpa", trel, fpa_deg)
                    self.fpa_latest = fpa_deg
                stride_idx = extras.get("stride")
                if stride_idx is not None:
                    try:
                        self.fpa_stride = int(stride_idx)
                    except (ValueError, TypeError):
                        self.fpa_stride = None
                dx = extras.get("dx")
                dy = extras.get("dy")
                stride_time = extras.get("ts")
                self.fpa_dx = dx if (dx is not None and math.isfinite(dx)) else math.nan
                self.fpa_dy = dy if (dy is not None and math.isfinite(dy)) else math.nan
                self.fpa_stride_time = stride_time if (stride_time is not None and math.isfinite(stride_time)) else math.nan

                roll_extra = extras.get("footroll_deg")
                if roll_extra is None:
                    roll_extra = extras.get("footroll")
                pitch_extra = extras.get("footpitch_deg")
                if pitch_extra is None:
                    pitch_extra = extras.get("footpitch")
                yaw_extra = extras.get("footyaw_deg")
                if yaw_extra is None:
                    yaw_extra = extras.get("footyaw")
                if (roll_extra is not None) or (pitch_extra is not None) or (yaw_extra is not None):
                    FOOT_ORIENTATION.update(
                        roll_extra if roll_extra is not None else math.nan,
                        pitch_extra if pitch_extra is not None else math.nan,
                        yaw_extra if yaw_extra is not None else math.nan,
                    )

        self.pull_latest_foot_orientation()
        self.update_foot_axes()
        self.update_rpy3d_window()

        # update numeric RPY labels so user always sees orientation even if GL fails
        try:
            if math.isfinite(self.foot_roll):
                self.rpy_lbl_roll.setText(f"Roll: {self.foot_roll:.1f}°")
            else:
                self.rpy_lbl_roll.setText("Roll: --.-°")
            if math.isfinite(self.foot_pitch):
                self.rpy_lbl_pitch.setText(f"Pitch: {self.foot_pitch:.1f}°")
            else:
                self.rpy_lbl_pitch.setText("Pitch: --.-°")
            if math.isfinite(self.foot_yaw):
                self.rpy_lbl_yaw.setText(f"Yaw: {self.foot_yaw:.1f}°")
            else:
                self.rpy_lbl_yaw.setText("Yaw: --.-°")
        except Exception:
            pass

        # update curves if we got anything
        if drained:
            # x-range: last window
            x0 = max(0.0, self.t_last - self.win)
            x1 = self.t_last if self.t_last > 0 else self.win

            # accel
            if self.plot_acc is not None:
                tx, axv = self.series("ax"); _, ayv = self.series("ay"); _, azv = self.series("az")
                self.cur_ax.setData(tx, axv)
                self.cur_ay.setData(tx, ayv)
                self.cur_az.setData(tx, azv)
                self.plot_acc.setXRange(x0, x1, padding=0.0)

            # gyro
            if self.plot_gyr is not None:
                tg, gxv = self.series("gx"); _, gyv = self.series("gy"); _, gzv = self.series("gz")
                self.cur_gx.setData(tg, gxv)
                self.cur_gy.setData(tg, gyv)
                self.cur_gz.setData(tg, gzv)
                self.plot_gyr.setXRange(x0, x1, padding=0.0)

            # mag
            if self.plot_mag is not None:
                tm, mxv = self.series("mx"); _, myv = self.series("my"); _, mzv = self.series("mz")
                self.cur_mx.setData(tm, mxv)
                self.cur_my.setData(tm, myv)
                self.cur_mz.setData(tm, mzv)
                self.plot_mag.setXRange(x0, x1, padding=0.0)

            # rpy
            if self.use_vqf and self.plot_rpy is not None:
                tr, rv = self.series("roll"); _, pv = self.series("pitch"); _, yv = self.series("yaw")
                self.cur_roll.setData(tr, rv)
                self.cur_pitch.setData(tr, pv)
                self.cur_yaw.setData(tr, yv)
                self.plot_rpy.setXRange(x0, x1, padding=0.0)

            if self.cur_fpa is not None:
                tf, fvp = self.series("fpa")
                self.cur_fpa.setData(tf, fvp)
                self.plot_fpa.setXRange(x0, x1, padding=0.0)

            # status
            if self._samples_key is not None and self._samples_key in self.buf:
                self.samples_in_window = len(self.buf[self._samples_key])
            else:
                self.samples_in_window = 0
            status = [
                f"Window: {self.win:.1f}s",
                f"samples: ~{self.samples_in_window}",
                f"VQF: {'ON' if self.use_vqf else 'OFF'}",
                f"gyro_units_in: {self.args.gyro_units}"
            ]
            if math.isfinite(self.fpa_latest):
                stride_txt = f"stride {self.fpa_stride}" if self.fpa_stride is not None else "stride ?"
                status.append(f"FPA: {self.fpa_latest:.1f}° ({stride_txt})")
                if math.isfinite(self.fpa_stride_time):
                    status.append(f"Ts={self.fpa_stride_time:.2f}s")
            if math.isfinite(self.foot_roll) and math.isfinite(self.foot_pitch) and math.isfinite(self.foot_yaw):
                status.append(f"Foot r/p/y={self.foot_roll:.1f}/{self.foot_pitch:.1f}/{self.foot_yaw:.1f}°")
            self.status.showMessage("  |  ".join(status))


def main():
    args = parse_args()
    app = QtWidgets.QApplication([])
    w = IMUWindow(args)
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
