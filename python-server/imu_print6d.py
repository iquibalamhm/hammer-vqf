#!/usr/bin/env python3
"""
IMU pyqtgraph visualizer (5s rolling window)
- Plots accel (m/s^2), gyro (deg/s), orientation (deg) from device quaternions
- Input: serial (--port) or file (--file)
- Expected format: t_ms,ax,ay,az,gx,gy,gz,qw,qx,qy,qz

Install:
  pip install pyqtgraph PyQt5 pyserial

Examples:
  python imu_print6d.py --port /dev/ttyACM0 --baud 115200 --gyro-units rad
  python imu_print6d.py --file sample.csv --gyro-units rad
  python imu_print6d.py --port /dev/ttyACM0 --rpy-3d
"""
import sys, math, time, threading, queue, argparse
import numpy as np
from collections import deque

# -------- optional deps ----------
try:
    import serial  # pyserial (only needed for --port)
except Exception:
    serial = None

try:
    from vqf import VQF
    HAVE_VQF = True
except Exception:
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
    ap.add_argument("--queue-max", type=int, default=10000, help="Reader→GUI queue size")
    ap.add_argument("--rpy-3d", action="store_true",
                    help="Show roll/pitch/yaw in a separate 3D window (requires pyqtgraph.opengl)")
    return ap.parse_args()


def parse_line(line):
    """
    Parse 't_ms,ax,ay,az,gx,gy,gz,qw,qx,qy,qz' → tuple
    Returns None if malformed.
    """
    s = line.strip()
    if not s or s.startswith("#"): return None
    parts = s.split(",")
    if len(parts) != 11: return None
    try:
        vals = [float("nan") if p.strip().lower()=="nan" else float(p) for p in parts]
    except ValueError:
        return None
    t_ms = vals[0]
    return (t_ms, *vals[1:])


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
        finally:
            self.close()


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


class IMUWindow(QtWidgets.QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.setWindowTitle("IMU Live (pyqtgraph)")
        self.resize(1200, 800)

        self.args = args
        self.win = args.window

        # Data buffers (deques of (t, value))
        self.t0 = None
        self.t_last = 0.0
        self.buf = {
            "ax": deque(), "ay": deque(), "az": deque(),
            "gx": deque(), "gy": deque(), "gz": deque(),
            "roll": deque(), "pitch": deque(), "yaw": deque(),
        }

        # Orientation is now provided directly as quaternions from the device
        self.use_quat = True  # Always true since device sends quaternions
        self.last_roll = math.nan
        self.last_pitch = math.nan
        self.last_yaw = math.nan

        self.rpy3d_window = None
        if args.rpy_3d:
            if not HAVE_GL:
                print("[imu_print6d9d] --rpy-3d requested but pyqtgraph.opengl is unavailable; ignoring.", file=sys.stderr)
            else:
                try:
                    self.rpy3d_window = RPY3DWindow(parent=self)
                    self.rpy3d_window.show()
                    self.rpy3d_window.raise_()
                except Exception as e:
                    print(f"[imu_print6d9d] Failed to initialize 3D RPY window: {e}", file=sys.stderr)
                    self.rpy3d_window = None

        # UI layout
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        vbox = QtWidgets.QVBoxLayout(central)
        self.glw = pg.GraphicsLayoutWidget(show=True)
        vbox.addWidget(self.glw)

        # Plots: Accel, Gyro, Mag, (optional) Orientation
        self.plot_acc = self.glw.addPlot(title="Acceleration (m/s²)")
        self.plot_acc.addLegend()
        self.plot_acc.showGrid(x=True, y=True, alpha=0.3)
        self.cur_ax = self.plot_acc.plot([], [], pen=pg.mkPen((255,120,120), width=2), name="ax")
        self.cur_ay = self.plot_acc.plot([], [], pen=pg.mkPen((120,255,120), width=2), name="ay")
        self.cur_az = self.plot_acc.plot([], [], pen=pg.mkPen((120,170,255), width=2), name="az")

        self.glw.nextRow()
        self.plot_gyr = self.glw.addPlot(title="Gyroscope (deg/s)")
        self.plot_gyr.addLegend()
        self.plot_gyr.showGrid(x=True, y=True, alpha=0.3)
        self.cur_gx = self.plot_gyr.plot([], [], pen=pg.mkPen((255,180,120), width=2), name="gx")
        self.cur_gy = self.plot_gyr.plot([], [], pen=pg.mkPen((180,255,120), width=2), name="gy")
        self.cur_gz = self.plot_gyr.plot([], [], pen=pg.mkPen((180,200,255), width=2), name="gz")

        # Orientation plot (quaternions from device converted to Euler)
        self.glw.nextRow()
        self.plot_rpy = self.glw.addPlot(title="Orientation (from device quaternion, deg)")
        self.plot_rpy.addLegend()
        self.plot_rpy.showGrid(x=True, y=True, alpha=0.3)
        self.plot_rpy.setYRange(-180, 180, padding=0.05)  # Set reasonable Y range for Euler angles
        self.cur_roll  = self.plot_rpy.plot([], [], pen=pg.mkPen((255,80,80), width=3), name="roll")
        self.cur_pitch = self.plot_rpy.plot([], [], pen=pg.mkPen((80,255,80), width=3), name="pitch")
        self.cur_yaw   = self.plot_rpy.plot([], [], pen=pg.mkPen((80,80,255), width=3), name="yaw")

        # Reader thread + timer to drain queue
        self.q = queue.Queue(maxsize=args.queue_max)
        self.stop_evt = threading.Event()
        self.reader = ReaderThread(args.port, args.baud, args.file, self.q, self.stop_evt)
        self.reader.start()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.on_timer)
        self.timer.start(int(1000 / max(1.0, args.ui_hz)))

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
        ev.accept()

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
        d = self.buf[key]
        d.append((t, val))
        # drop older than window
        t_cut = t - self.win
        while d and d[0][0] < t_cut:
            d.popleft()

    def series(self, key):
        d = self.buf[key]
        if not d: return [], []
        ts = [t for (t, _) in d]
        vs = [v for (_, v) in d]
        return ts, vs

    def update_rpy3d_window(self, roll=None, pitch=None, yaw=None):
        if self.rpy3d_window is None:
            return
        r = self.last_roll if roll is None else roll
        p = self.last_pitch if pitch is None else pitch
        y = self.last_yaw if yaw is None else yaw
        try:
            self.rpy3d_window.set_orientation(r, p, y)
        except Exception as exc:
            print(f"[imu_print6d9d] Failed to update RPY 3D window: {exc}", file=sys.stderr)

    def on_timer(self):
        # drain queue
        drained = 0
        while True:
            try:
                rec = self.q.get_nowait()
            except queue.Empty:
                break
            drained += 1
            t_ms, ax, ay, az, gx, gy, gz, qw, qx, qy, qz = rec
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
            else:
                gxd = gx; gyd = gy; gzd = gz
            self.append("gx", trel, gxd)
            self.append("gy", trel, gyd)
            self.append("gz", trel, gzd)

            # Convert quaternion to Euler angles (device sends w,x,y,z)
            if not (math.isnan(qw) or math.isnan(qx) or math.isnan(qy) or math.isnan(qz)):
                roll, pitch, yaw = [math.degrees(v) for v in self.quat_to_euler_zyx(qw, qx, qy, qz)]
                # Debug: print first few samples
                if drained <= 3:
                    print(f"[DEBUG] quat: w={qw:.4f} x={qx:.4f} y={qy:.4f} z={qz:.4f} -> r={roll:.2f}° p={pitch:.2f}° y={yaw:.2f}°")
                self.append("roll", trel, roll)
                self.append("pitch", trel, pitch)
                self.append("yaw", trel, yaw)
                self.last_roll = roll
                self.last_pitch = pitch
                self.last_yaw = yaw
                self.update_rpy3d_window(roll, pitch, yaw)
            else:
                # If quaternion is invalid, append NaN to keep time series aligned
                if drained <= 3:
                    print(f"[DEBUG] Invalid quaternion: w={qw} x={qx} y={qy} z={qz}")
                self.append("roll", trel, math.nan)
                self.append("pitch", trel, math.nan)
                self.append("yaw", trel, math.nan)

        # update curves if we got anything
        if drained:
            # x-range: last window
            x0 = max(0.0, self.t_last - self.win)
            x1 = self.t_last if self.t_last > 0 else self.win

            # accel
            tx, axv = self.series("ax"); _, ayv = self.series("ay"); _, azv = self.series("az")
            self.cur_ax.setData(tx, axv); self.cur_ay.setData(tx, ayv); self.cur_az.setData(tx, azv)
            self.plot_acc.setXRange(x0, x1, padding=0.0)

            # gyro
            tg, gxv = self.series("gx"); _, gyv = self.series("gy"); _, gzv = self.series("gz")
            self.cur_gx.setData(tg, gxv); self.cur_gy.setData(tg, gyv); self.cur_gz.setData(tg, gzv)
            self.plot_gyr.setXRange(x0, x1, padding=0.0)

            # rpy (from quaternions sent by device)
            tr, rv = self.series("roll")
            tp, pv = self.series("pitch")
            ty, yv = self.series("yaw")
            
            # Debug: print buffer info once
            if self.samples_in_window == 0 and len(rv) > 0:
                print(f"[DEBUG] First RPY update: {len(rv)} samples, roll range: [{min(rv):.2f}, {max(rv):.2f}]")
            
            # Filter out NaN values for plotting
            if len(rv) > 0:
                tr_valid = [t for t, v in zip(tr, rv) if math.isfinite(v)]
                rv_valid = [v for v in rv if math.isfinite(v)]
                pv_valid = [v for v in pv if math.isfinite(v)]
                yv_valid = [v for v in yv if math.isfinite(v)]
                
                if len(tr_valid) > 0:
                    self.cur_roll.setData(tr_valid, rv_valid)
                    self.cur_pitch.setData(tr_valid, pv_valid)
                    self.cur_yaw.setData(tr_valid, yv_valid)
            
            self.plot_rpy.setXRange(x0, x1, padding=0.0)

            # status
            self.samples_in_window = len(self.buf["ax"])
            self.status.showMessage(
                f"Window: {self.win:.1f}s  |  samples: ~{self.samples_in_window}  |  "
                f"Orientation: from device quaternion  |  gyro_units_in: {self.args.gyro_units}"
            )


def main():
    args = parse_args()
    app = QtWidgets.QApplication([])
    w = IMUWindow(args)
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
