#!/usr/bin/env python3
"""
IMU pyqtgraph visualizer (5s rolling window)
- Plots accel (m/s^2), gyro (deg/s), mag (uT)
- Optional VQF roll/pitch/yaw (deg) if `vqf` is installed
- Input: serial (--port) or file (--file)

Install:
  pip install pyqtgraph PyQt5 pyserial vqf

Examples:
  python imu_tui.py --port /dev/ttyACM0 --baud 115200 --gyro-units rad
  python imu_tui.py --file sample.csv --gyro-units rad
  python imu_tui.py --file sample.csv --no-orientation
"""
import sys, math, time, threading, queue, argparse
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
    return ap.parse_args()


def parse_line(line):
    """
    Parse 't_ms,ax,ay,az,gx,gy,gz,mx,my,mz' → tuple
    Returns None if malformed.
    """
    s = line.strip()
    if not s or s.startswith("#"): return None
    parts = s.split(",")
    if len(parts) != 10: return None
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
            "mx": deque(), "my": deque(), "mz": deque(),
            "roll": deque(), "pitch": deque(), "yaw": deque(),
        }

        # Orientation (optional)
        self.use_vqf = (not args.no_orientation) and HAVE_VQF
        self.vqf = VQF() if self.use_vqf else None
        self.prev_t = None

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

        self.glw.nextRow()
        self.plot_mag = self.glw.addPlot(title="Magnetometer (µT)")
        self.plot_mag.addLegend()
        self.plot_mag.showGrid(x=True, y=True, alpha=0.3)
        self.cur_mx = self.plot_mag.plot([], [], pen=pg.mkPen((255,220,120), width=2), name="mx")
        self.cur_my = self.plot_mag.plot([], [], pen=pg.mkPen((160,255,200), width=2), name="my")
        self.cur_mz = self.plot_mag.plot([], [], pen=pg.mkPen((200,200,255), width=2), name="mz")

        if self.use_vqf:
            self.glw.nextRow()
            self.plot_rpy = self.glw.addPlot(title="Orientation (VQF, deg)")
            self.plot_rpy.addLegend()
            self.plot_rpy.showGrid(x=True, y=True, alpha=0.3)
            self.cur_roll  = self.plot_rpy.plot([], [], pen=pg.mkPen((255,140,140), width=2), name="roll")
            self.cur_pitch = self.plot_rpy.plot([], [], pen=pg.mkPen((140,255,140), width=2), name="pitch")
            self.cur_yaw   = self.plot_rpy.plot([], [], pen=pg.mkPen((140,140,255), width=2), name="yaw")

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

    def on_timer(self):
        # drain queue
        drained = 0
        while True:
            try:
                rec = self.q.get_nowait()
            except queue.Empty:
                break
            drained += 1
            t_ms, ax, ay, az, gx, gy, gz, mx, my, mz = rec
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

            # mag
            tm, mxv = self.series("mx"); _, myv = self.series("my"); _, mzv = self.series("mz")
            self.cur_mx.setData(tm, mxv); self.cur_my.setData(tm, myv); self.cur_mz.setData(tm, mzv)
            self.plot_mag.setXRange(x0, x1, padding=0.0)

            # rpy
            if self.use_vqf:
                tr, rv = self.series("roll"); _, pv = self.series("pitch"); _, yv = self.series("yaw")
                self.cur_roll.setData(tr, rv); self.cur_pitch.setData(tr, pv); self.cur_yaw.setData(tr, yv)
                self.plot_rpy.setXRange(x0, x1, padding=0.0)

            # status
            self.samples_in_window = len(self.buf["ax"])
            self.status.showMessage(
                f"Window: {self.win:.1f}s  |  samples: ~{self.samples_in_window}  |  "
                f"VQF: {'ON' if self.use_vqf else 'OFF'}  |  gyro_units_in: {self.args.gyro_units}"
            )


def main():
    args = parse_args()
    app = QtWidgets.QApplication([])
    w = IMUWindow(args)
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
