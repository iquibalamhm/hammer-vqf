import sys
import math
import threading
import queue
from collections import deque

import numpy as np

import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore

try:
	import pyqtgraph.opengl as gl
	HAVE_GL = True
except Exception:
	gl = None
	HAVE_GL = False

from .fpa_events import FootFlatDetector
from .reader import ReaderThread
from .hrfc import HRFCDetector


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
	def __init__(self, args, parse_fn, reader_cls=ReaderThread):
		super().__init__()
		if parse_fn is None:
			raise ValueError("parse_fn is required for IMUWindow")
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
			"qw": deque(), "qx": deque(), "qy": deque(), "qz": deque(),
		}
		
		# HR/FC detection buffers
		self.hrfc_buf = {
			"t": deque(),
			"a_err": deque(),
			"a_th": deque(),
			"w_norm": deque(),
			"w_th": deque(),
			"r": deque(),
		}
		self.hr_points = []  # [(t, y), ...]
		self.fc_points = []  # [(t, y), ...]

		# Foot-flat detector (expects gyro in rad/s)
		self.det = FootFlatDetector(
			g=self.args.g,
			win_thresh_s=self.args.th_win,
			hyst_frac=self.args.hyst_frac,
			a_th_min=self.args.a_th_min,
			w_th_min=self.args.w_th_min,
			t0_min=self.args.t0_min,
			t1_min=self.args.t1_min,
			rc_iters=self.args.rc_iters,
			w_acc=self.args.w_acc,
			w_gyr=self.args.w_gyr,
		)
		
		# HR/FC detector
		self.hrfc_det = HRFCDetector(
			g=self.args.g,
			th_win=self.args.th_win,
			rc_iters=self.args.rc_iters,
			hyst_frac=self.args.hyst_frac,
			a_th_min=self.args.a_th_min,
			w_th_min=self.args.w_th_min,
			w_acc=self.args.w_acc,
			w_gyr=self.args.w_gyr,
			T0_min=self.args.t0_min,
			T1_min=self.args.t1_min,
			gyro_units=self.args.gyro_units
		)

		# Gyro unit conversion for detector (rad/s)
		self._to_rad = (math.pi/180.0) if self.args.gyro_units == "deg" else 1.0

		# Event marker handles
		self._fc_lines = []
		self._hr_lines = []
		self._rest_lines = []
		
		# Debug counters
		self.sample_count = 0
		self.debug_interval = 100  # Print debug info every 100 samples

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
		self.plot_acc.setTitle("Acceleration (m/s²) — FC(green)  HR(red)  REST(blue dashed)")
		self.cur_ax = self.plot_acc.plot([], [], pen=pg.mkPen((255,120,120), width=2), name="ax")
		self.cur_ay = self.plot_acc.plot([], [], pen=pg.mkPen((120,255,120), width=2), name="ay")
		self.cur_az = self.plot_acc.plot([], [], pen=pg.mkPen((120,170,255), width=2), name="az")
		# HR/FC overlays: |a|-g and threshold
		self.cur_a_err = self.plot_acc.plot([], [], pen=pg.mkPen((255,255,100), width=2), name="|a|-g")
		self.cur_a_th = self.plot_acc.plot([], [], pen=pg.mkPen((255,150,0), width=2, style=QtCore.Qt.DashLine), name="a_th")

		self.glw.nextRow()
		self.plot_gyr = self.glw.addPlot(title="Gyroscope (deg/s)")
		self.plot_gyr.addLegend()
		self.plot_gyr.showGrid(x=True, y=True, alpha=0.3)
		self.cur_gx = self.plot_gyr.plot([], [], pen=pg.mkPen((255,180,120), width=2), name="gx")
		self.cur_gy = self.plot_gyr.plot([], [], pen=pg.mkPen((180,255,120), width=2), name="gy")
		self.cur_gz = self.plot_gyr.plot([], [], pen=pg.mkPen((180,200,255), width=2), name="gz")
		# HR/FC overlays: ‖ω‖ and threshold
		self.cur_w_norm = self.plot_gyr.plot([], [], pen=pg.mkPen((255,255,100), width=2), name="‖ω‖")
		self.cur_w_th = self.plot_gyr.plot([], [], pen=pg.mkPen((255,150,0), width=2, style=QtCore.Qt.DashLine), name="w_th")

		# Quaternion plot (raw quaternion components from device)
		self.glw.nextRow()
		self.plot_quat = self.glw.addPlot(title="Quaternion (from device)")
		self.plot_quat.addLegend()
		self.plot_quat.showGrid(x=True, y=True, alpha=0.3)
		self.plot_quat.setYRange(-1.1, 1.1, padding=0.05)
		self.cur_qw = self.plot_quat.plot([], [], pen=pg.mkPen((255,100,255), width=2), name="qw")
		self.cur_qx = self.plot_quat.plot([], [], pen=pg.mkPen((255,120,120), width=2), name="qx")
		self.cur_qy = self.plot_quat.plot([], [], pen=pg.mkPen((120,255,120), width=2), name="qy")
		self.cur_qz = self.plot_quat.plot([], [], pen=pg.mkPen((120,170,255), width=2), name="qz")

		# Orientation plot (quaternions from device converted to Euler)
		self.glw.nextRow()
		self.plot_rpy = self.glw.addPlot(title="Orientation (from device quaternion, deg)")
		self.plot_rpy.addLegend()
		self.plot_rpy.showGrid(x=True, y=True, alpha=0.3)
		self.plot_rpy.setYRange(-180, 180, padding=0.05)
		self.cur_roll  = self.plot_rpy.plot([], [], pen=pg.mkPen((255,80,80), width=3), name="roll")
		self.cur_pitch = self.plot_rpy.plot([], [], pen=pg.mkPen((80,255,80), width=3), name="pitch")
		self.cur_yaw   = self.plot_rpy.plot([], [], pen=pg.mkPen((80,80,255), width=3), name="yaw")
		
		# Event strip: rest signal + HR/FC markers
		self.glw.nextRow()
		self.plot_events = self.glw.addPlot(title="Gait Events (0=move/HR, 1=rest/FC)")
		self.plot_events.addLegend()
		self.plot_events.showGrid(x=True, y=True, alpha=0.3)
		self.plot_events.setYRange(-0.2, 1.2, padding=0.0)
		self.cur_rest = self.plot_events.plot([], [], pen=pg.mkPen((150,150,150), width=3), 
		                                      name="rest r(t)", connect='finite')
		self.hr_scatter = pg.ScatterPlotItem(size=12, pen=pg.mkPen((220,0,0), width=2), 
		                                     brush=pg.mkBrush((220,0,0,150)), 
		                                     symbol='t1', name="HR")
		self.fc_scatter = pg.ScatterPlotItem(size=12, pen=pg.mkPen((0,200,0), width=2), 
		                                     brush=pg.mkBrush((0,200,0,150)), 
		                                     symbol='t', name="FC")
		self.plot_events.addItem(self.hr_scatter)
		self.plot_events.addItem(self.fc_scatter)
		self.plot_events.addLegend()

		# Reader thread + timer to drain queue
		self.q = queue.Queue(maxsize=args.queue_max)
		self.stop_evt = threading.Event()
		self.reader = reader_cls(args.port, args.baud, args.file, self.q, self.stop_evt, parse_fn=parse_fn)
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

	@staticmethod
	def quat_to_euler_zyx(w, x, y, z):
		# ZYX: yaw(Z), pitch(Y), roll(X)
		s = max(min(2*(w*y - z*x), 1.0), -1.0)
		yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
		pitch = math.asin(s)
		roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
		return roll, pitch, yaw

	def append(self, key, t, val):
		d = self.buf[key]
		d.append((t, val))
		t_cut = t - self.win
		while d and d[0][0] < t_cut:
			d.popleft()

	def series(self, key):
		d = self.buf[key]
		if not d:
			return [], []
		ts = [t for (t, _) in d]
		vs = [v for (_, v) in d]
		return ts, vs

	def _add_event_marker(self, t_rel, tag):
		if tag == "FC":
			pen = pg.mkPen((0,200,0), width=2)
			line = pg.InfiniteLine(pos=t_rel, angle=90, movable=False, pen=pen)
			self._fc_lines.append(line)
		elif tag == "HR":
			pen = pg.mkPen((220,0,0), width=2)
			line = pg.InfiniteLine(pos=t_rel, angle=90, movable=False, pen=pen)
			self._hr_lines.append(line)
		else:
			pen = pg.mkPen((50,120,255), width=2, style=QtCore.Qt.DashLine)
			line = pg.InfiniteLine(pos=t_rel, angle=90, movable=False, pen=pen)
			self._rest_lines.append(line)
		for p in (self.plot_acc, self.plot_gyr, self.plot_quat, self.plot_rpy, self.plot_events):
			p.addItem(line)

	def _prune_old_markers(self, x_min):
		def _keep(lines):
			kept = []
			for ln in lines:
				t = ln.value() if hasattr(ln, "value") else ln.pos().x()
				if t >= x_min:
					kept.append(ln)
				else:
					for p in (self.plot_acc, self.plot_gyr, self.plot_quat, self.plot_rpy, self.plot_events):
						try:
							p.removeItem(ln)
						except Exception:
							pass
			return kept

		self._fc_lines = _keep(self._fc_lines)
		self._hr_lines = _keep(self._hr_lines)
		self._rest_lines = _keep(self._rest_lines)

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

			self.append("ax", trel, ax)
			self.append("ay", trel, ay)
			self.append("az", trel, az)

			if self.args.gyro_units == "rad":
				gxd = math.degrees(gx)
				gyd = math.degrees(gy)
				gzd = math.degrees(gz)
			else:
				gxd = gx
				gyd = gy
				gzd = gz
			self.append("gx", trel, gxd)
			self.append("gy", trel, gyd)
			self.append("gz", trel, gzd)

			gx_rad = gx * self._to_rad
			gy_rad = gy * self._to_rad
			gz_rad = gz * self._to_rad

			events = self.det.update(t_ms, ax, ay, az, gx_rad, gy_rad, gz_rad)
			for tag, t_evt_ms in events:
				t_evt_rel = (t_evt_ms * 1e-3) - self.t0
				self._add_event_marker(t_evt_rel, tag)
				print(f"EVT,{t_evt_ms:.3f},{tag}")
			
			# HR/FC detector update
			self.hrfc_det.update(t_ms, ax, ay, az, gx, gy, gz, qw, qx, qy, qz)
			dbg = self.hrfc_det.get_debug()
			
			# Sample counter and periodic debug
			self.sample_count += 1
			if self.sample_count % self.debug_interval == 0:
				print(f"[DEBUG] Sample {self.sample_count}: t={t_ms:.0f}ms, "
				      f"a_err={dbg['a_err']:.3f}, a_th={dbg['a_th']:.3f}, "
				      f"w_norm={dbg['w_norm']:.4f}, w_th={dbg['w_th']:.4f}, "
				      f"rest={dbg['r']}")
				sys.stdout.flush()
			
			# Store HR/FC debug data
			self.hrfc_buf["t"].append(trel)
			self.hrfc_buf["a_err"].append(dbg['a_err'])
			self.hrfc_buf["a_th"].append(dbg['a_th'])
			self.hrfc_buf["w_norm"].append(dbg['w_norm'])
			self.hrfc_buf["w_th"].append(dbg['w_th'])
			self.hrfc_buf["r"].append(dbg['r'])
			
			# Trim HR/FC buffers
			t_cut = trel - self.win
			for key in self.hrfc_buf:
				buf = self.hrfc_buf[key]
				while buf and (self.hrfc_buf["t"][0] if key == "t" else True) and \
				      len(self.hrfc_buf["t"]) > 0 and self.hrfc_buf["t"][0] < t_cut:
					for k in self.hrfc_buf:
						if self.hrfc_buf[k]:
							self.hrfc_buf[k].popleft()
					break
			
			# Pop HR/FC events
			for tag, te in self.hrfc_det.pop_events():
				t_evt_rel = te - self.t0
				if tag == "HR":
					self.hr_points.append((t_evt_rel, 0.0))
					print(f"EVT,{te:.3f},{tag} (HR - Heel Rise)")
					sys.stdout.flush()
				else:  # FC
					self.fc_points.append((t_evt_rel, 1.0))
					print(f"EVT,{te:.3f},{tag} (FC - Full Contact)")
					sys.stdout.flush()

			# Store quaternion components
			self.append("qw", trel, qw)
			self.append("qx", trel, qx)
			self.append("qy", trel, qy)
			self.append("qz", trel, qz)

			if not (math.isnan(qw) or math.isnan(qx) or math.isnan(qy) or math.isnan(qz)):
				roll, pitch, yaw = [math.degrees(v) for v in self.quat_to_euler_zyx(qw, qx, qy, qz)]
				self.append("roll", trel, roll)
				self.append("pitch", trel, pitch)
				self.append("yaw", trel, yaw)
				self.last_roll = roll
				self.last_pitch = pitch
				self.last_yaw = yaw
				self.update_rpy3d_window(roll, pitch, yaw)
			else:
				self.append("roll", trel, math.nan)
				self.append("pitch", trel, math.nan)
				self.append("yaw", trel, math.nan)

		if drained:
			x0 = max(0.0, self.t_last - self.win)
			x1 = self.t_last if self.t_last > 0 else self.win

			tx, axv = self.series("ax")
			_, ayv = self.series("ay")
			_, azv = self.series("az")
			self.cur_ax.setData(tx, axv)
			self.cur_ay.setData(tx, ayv)
			self.cur_az.setData(tx, azv)
			
			# HR/FC accel overlays
			t_hrfc = list(self.hrfc_buf["t"])
			a_err_v = list(self.hrfc_buf["a_err"])
			a_th_v = list(self.hrfc_buf["a_th"])
			self.cur_a_err.setData(t_hrfc, a_err_v)
			self.cur_a_th.setData(t_hrfc, a_th_v)
			
			self.plot_acc.setXRange(x0, x1, padding=0.0)

			tg, gxv = self.series("gx")
			_, gyv = self.series("gy")
			_, gzv = self.series("gz")
			self.cur_gx.setData(tg, gxv)
			self.cur_gy.setData(tg, gyv)
			self.cur_gz.setData(tg, gzv)
			
			# HR/FC gyro overlays (convert to deg/s for display if needed)
			w_norm_v = list(self.hrfc_buf["w_norm"])
			w_th_v = list(self.hrfc_buf["w_th"])
			if self.args.gyro_units == "rad":
				w_norm_v = [math.degrees(w) for w in w_norm_v]
				w_th_v = [math.degrees(w) for w in w_th_v]
			self.cur_w_norm.setData(t_hrfc, w_norm_v)
			self.cur_w_th.setData(t_hrfc, w_th_v)
			
			self.plot_gyr.setXRange(x0, x1, padding=0.0)

			tq, qwv = self.series("qw")
			_, qxv = self.series("qx")
			_, qyv = self.series("qy")
			_, qzv = self.series("qz")
			self.cur_qw.setData(tq, qwv)
			self.cur_qx.setData(tq, qxv)
			self.cur_qy.setData(tq, qyv)
			self.cur_qz.setData(tq, qzv)
			self.plot_quat.setXRange(x0, x1, padding=0.0)

			tr, rv = self.series("roll")
			tp, pv = self.series("pitch")
			ty, yv = self.series("yaw")

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
			
			# Event strip: rest signal + HR/FC markers
			r_v = list(self.hrfc_buf["r"])
			self.cur_rest.setData(t_hrfc, r_v)
			self.plot_events.setXRange(x0, x1, padding=0.0)
			
			# Filter and update HR/FC scatter points
			hr_filtered = [(t, y) for (t, y) in self.hr_points if t >= x0]
			fc_filtered = [(t, y) for (t, y) in self.fc_points if t >= x0]
			
			# Trim old points
			self.hr_points = hr_filtered
			self.fc_points = fc_filtered
			
			if hr_filtered:
				hrx, hry = zip(*hr_filtered)
				self.hr_scatter.setData(hrx, hry)
			else:
				self.hr_scatter.setData([], [])
			
			if fc_filtered:
				fcx, fcy = zip(*fc_filtered)
				self.fc_scatter.setData(fcx, fcy)
			else:
				self.fc_scatter.setData([], [])

			self._prune_old_markers(x0)

			self.samples_in_window = len(self.buf["ax"])
			self.status.showMessage(
				f"Window: {self.win:.1f}s  |  samples: ~{self.samples_in_window}  |  "
				f"Orientation: from device quaternion  |  gyro_units_in: {self.args.gyro_units}"
			)


__all__ = ["ReaderThread", "RPY3DWindow", "IMUWindow", "HAVE_GL"]
