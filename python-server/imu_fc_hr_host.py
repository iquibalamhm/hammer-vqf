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

  python imu_fc_hr_host.py --port /dev/ttyACM0 --baud 115200 --gyro-units rad --rpy-3d
"""
import sys
import argparse

from PyQt5 import QtWidgets

from utils.fpa_events import G_DEFAULT
from utils.plotter import IMUWindow


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
                    help="Show roll/pitch/yaw in a separate 3D window")

    # --- Foot-flat detector (3.1–3.4) params ---
    ap.add_argument("--g", type=float, default=G_DEFAULT, help="Gravity for |‖a‖-g|")
    ap.add_argument("--th-win", type=float, default=5.0, help="Threshold-estimation window (sec)")
    ap.add_argument("--rc-iters", type=int, default=100, help="Iterative threshold iterations")
    ap.add_argument("--hyst-frac", type=float, default=0.23, help="Schmitt hysteresis fraction")
    ap.add_argument("--a-th-min", type=float, default=0.5, help="Min accel threshold (m/s^2)")
    ap.add_argument("--w-th-min", type=float, default=0.2, help="Min gyro threshold (rad/s)")
    ap.add_argument("--t0-min", type=float, default=0.120, help="Min 0-run duration (sec)")
    ap.add_argument("--t1-min", type=float, default=0.180, help="Min 1-run duration (sec)")
    ap.add_argument("--w-acc", type=float, default=0.85, help="RC weight for accel")
    ap.add_argument("--w-gyr", type=float, default=0.80, help="RC weight for gyro")
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


def main():
    args = parse_args()
    app = QtWidgets.QApplication([])
    w = IMUWindow(args, parse_line)
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
