# Plotter per user's request:
# 1) All accelerations together + colored event markers
# 2) All gyros together + colored event markers
# 3) Gyro norm + colored event markers
#
# Reads the uploaded /mnt/data/csv_log.csv which mixes numeric rows and 'EVT,...' rows.
# Uses matplotlib (no seaborn). One chart per figure. We set colors because the user explicitly asked for colored events.
import csv
from pathlib import Path
import math
import numpy as np
import matplotlib.pyplot as plt
import argparse

# Parse command-line arguments
parser = argparse.ArgumentParser(description='Plot IMU data with optional time range filtering')
parser.add_argument('--csv', type=str, default='csv_log.csv', help='Path to CSV file (default: csv_log.csv)')
parser.add_argument('--tstart', type=float, default=None, help='Start time in ms (default: plot from beginning)')
parser.add_argument('--tend', type=float, default=None, help='End time in ms (default: plot to end)')
args = parser.parse_args()

csv_path = Path(args.csv)
assert csv_path.exists(), f"Could not find {csv_path}"

def maybe_float(x):
    try:
        return float(x)
    except Exception:
        return None

# Parse
t_list, ax_list, ay_list, az_list = [], [], [], []
gx_list, gy_list, gz_list = [], [], []
events = []  # dicts with {'foot','type','t'}
trajectories = []  # dicts with trajectory data
tags = {"FC","HR","TO","REST","REST_PRED","IC"}

with csv_path.open("r", newline="") as f:
    reader = csv.reader(f)
    for row in reader:
        if not row:
            continue
        row = [c.strip() for c in row]
        # Trajectory row? Format: TRJ,foot,t,v_raw(3),v_df(3),drift(3),pos(3)
        if len(row) >= 2 and row[0].upper() == "TRJ":
            vals = [maybe_float(c) for c in row[1:]]
            # Extract foot (should be row[1])
            foot = row[1] if len(row) > 1 else "?"
            # Then we have t and 12 float values
            floats = [v for v in vals if isinstance(v, float) and math.isfinite(v)]
            if len(floats) >= 13:  # t + 12 values
                t_traj = floats[0]
                vraw_x, vraw_y, vraw_z = floats[1:4]
                vdf_x, vdf_y, vdf_z = floats[4:7]
                drift_x, drift_y, drift_z = floats[7:10]
                pos_x, pos_y, pos_z = floats[10:13]
                trajectories.append({
                    "foot": foot, "t": t_traj,
                    "vraw_x": vraw_x, "vraw_y": vraw_y, "vraw_z": vraw_z,
                    "vdf_x": vdf_x, "vdf_y": vdf_y, "vdf_z": vdf_z,
                    "drift_x": drift_x, "drift_y": drift_y, "drift_z": drift_z,
                    "pos_x": pos_x, "pos_y": pos_y, "pos_z": pos_z
                })
            continue
        # Event row?
        if len(row) >= 2 and any("evt" in c.lower() for c in row):
            # timestamp = last numeric cell
            t = None
            for c in reversed(row):
                v = maybe_float(c)
                if v is not None:
                    t = v; break
            foot = "?"; etype = None
            for c in row:
                uc = c.upper()
                if uc in ("L","R"): foot = uc
                if uc in tags: etype = uc
            if (t is not None) and (etype is not None):
                events.append({"foot": foot, "type": etype, "t": t})
            continue
        # Numeric row expected: t, ax, ay, az, gx, gy, gz
        vals = [maybe_float(c) for c in row]
        floats = [v for v in vals if isinstance(v, float) and math.isfinite(v)]
        if len(floats) >= 7:
            t, ax, ay, az, gx, gy, gz = floats[:7]
            t_list.append(t); ax_list.append(ax); ay_list.append(ay); az_list.append(az)
            gx_list.append(gx); gy_list.append(gy); gz_list.append(gz)

# Convert to numpy and sort by time
t = np.array(t_list, float)
order = np.argsort(t)
t = t[order]
ax = np.array(ax_list, float)[order]
ay = np.array(ay_list, float)[order]
az = np.array(az_list, float)[order]
gx = np.array(gx_list, float)[order]
gy = np.array(gy_list, float)[order]
gz = np.array(gz_list, float)[order]

# Gyro norm
g_norm = np.sqrt(gx*gx + gy*gy + gz*gz)

# Convert trajectories to arrays
if trajectories:
    t_traj = np.array([tr["t"] for tr in trajectories])
    traj_order = np.argsort(t_traj)
    t_traj = t_traj[traj_order]
    
    vraw_x = np.array([trajectories[i]["vraw_x"] for i in traj_order])
    vraw_y = np.array([trajectories[i]["vraw_y"] for i in traj_order])
    vraw_z = np.array([trajectories[i]["vraw_z"] for i in traj_order])
    
    vdf_x = np.array([trajectories[i]["vdf_x"] for i in traj_order])
    vdf_y = np.array([trajectories[i]["vdf_y"] for i in traj_order])
    vdf_z = np.array([trajectories[i]["vdf_z"] for i in traj_order])
    
    drift_x = np.array([trajectories[i]["drift_x"] for i in traj_order])
    drift_y = np.array([trajectories[i]["drift_y"] for i in traj_order])
    drift_z = np.array([trajectories[i]["drift_z"] for i in traj_order])
    
    pos_x = np.array([trajectories[i]["pos_x"] for i in traj_order])
    pos_y = np.array([trajectories[i]["pos_y"] for i in traj_order])
    pos_z = np.array([trajectories[i]["pos_z"] for i in traj_order])

# Apply time range filtering if specified
if args.tstart is not None or args.tend is not None:
    tstart = args.tstart if args.tstart is not None else t.min()
    tend = args.tend if args.tend is not None else t.max()
    mask = (t >= tstart) & (t <= tend)
    t = t[mask]
    ax = ax[mask]
    ay = ay[mask]
    az = az[mask]
    gx = gx[mask]
    gy = gy[mask]
    gz = gz[mask]
    g_norm = g_norm[mask]
    # Filter events too
    events = [e for e in events if tstart <= e["t"] <= tend]
    # Filter trajectories
    if trajectories:
        traj_mask = (t_traj >= tstart) & (t_traj <= tend)
        t_traj = t_traj[traj_mask]
        vraw_x = vraw_x[traj_mask]; vraw_y = vraw_y[traj_mask]; vraw_z = vraw_z[traj_mask]
        vdf_x = vdf_x[traj_mask]; vdf_y = vdf_y[traj_mask]; vdf_z = vdf_z[traj_mask]
        drift_x = drift_x[traj_mask]; drift_y = drift_y[traj_mask]; drift_z = drift_z[traj_mask]
        pos_x = pos_x[traj_mask]; pos_y = pos_y[traj_mask]; pos_z = pos_z[traj_mask]
    print(f"Filtering data to time range [{tstart:.3f}, {tend:.3f}] ms")
    print(f"Kept {len(t)} data points, {len(events)} events, {len(t_traj) if trajectories else 0} trajectory points")
else:
    print(f"Plotting all data: {len(t)} points, {len(events)} events, {len(trajectories)} trajectory points")
    print(f"Time range: [{t.min():.3f}, {t.max():.3f}] ms")

# Colors for events
evt_colors = {
    "FC": "tab:green",
    "HR": "tab:orange",
    "TO": "tab:red",
    "REST": "tab:blue",
    "REST_PRED": "tab:purple",
    "IC": "tab:brown",
}

def draw_event_lines(axh):
    # group times by type so we can make a single legend entry per type
    by_type = {}
    for e in events:
        by_type.setdefault(e["type"], []).append(e["t"])
    for etype, times in by_type.items():
        c = evt_colors.get(etype, "k")
        for tt in times:
            axh.axvline(tt, color=c, linestyle="--", linewidth=1.0, alpha=0.9)
        # add a representative (invisible) for legend
        axh.axvline(times[len(times)//2], color=c, linestyle="--", linewidth=1.5, label=etype)

# 1) All accelerations together
plt.figure()
plt.plot(t, ax, label="ax")
plt.plot(t, ay, label="ay")
plt.plot(t, az, label="az")
draw_event_lines(plt.gca())
plt.title("Accelerations with Event Markers")
plt.xlabel("Time (ms)")
plt.ylabel("Acceleration")
plt.legend()
plt.tight_layout()

# 2) All gyros together
plt.figure()
plt.plot(t, gx, label="gx")
plt.plot(t, gy, label="gy")
plt.plot(t, gz, label="gz")
draw_event_lines(plt.gca())
plt.title("Gyroscope Components with Event Markers")
plt.xlabel("Time (ms)")
plt.ylabel("Angular rate")
plt.legend()
plt.tight_layout()

# 3) Gyro norm
plt.figure()
plt.plot(t, g_norm, label="|gyro|")
draw_event_lines(plt.gca())
plt.title("Gyroscope Norm with Event Markers")
plt.xlabel("Time (ms)")
plt.ylabel("|ω|")
plt.legend()
plt.tight_layout()

# Trajectory plots (if available)
if trajectories and len(t_traj) > 0:
    # 4) Position trajectory (x, y, z vs time)
    plt.figure()
    plt.plot(t_traj, pos_x, label="pos_x")
    plt.plot(t_traj, pos_y, label="pos_y")
    plt.plot(t_traj, pos_z, label="pos_z")
    draw_event_lines(plt.gca())
    plt.title("Position Trajectory with Event Markers")
    plt.xlabel("Time (ms)")
    plt.ylabel("Position (m)")
    plt.legend()
    plt.tight_layout()
    
    # 5) 2D position plot (x-y plane)
    plt.figure()
    plt.plot(pos_x, pos_y, 'b-', linewidth=1.5, label="trajectory")
    plt.plot(pos_x[0], pos_y[0], 'go', markersize=10, label="start")
    plt.plot(pos_x[-1], pos_y[-1], 'ro', markersize=10, label="end")
    plt.title("2D Position Trajectory (Top View)")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    
    # 6) Velocity (drift-free) components
    plt.figure()
    plt.plot(t_traj, vdf_x, label="v_df_x")
    plt.plot(t_traj, vdf_y, label="v_df_y")
    plt.plot(t_traj, vdf_z, label="v_df_z")
    draw_event_lines(plt.gca())
    plt.title("Drift-Free Velocity with Event Markers")
    plt.xlabel("Time (ms)")
    plt.ylabel("Velocity (m/s)")
    plt.legend()
    plt.tight_layout()
    
    # 7) Raw velocity components
    plt.figure()
    plt.plot(t_traj, vraw_x, label="v_raw_x")
    plt.plot(t_traj, vraw_y, label="v_raw_y")
    plt.plot(t_traj, vraw_z, label="v_raw_z")
    draw_event_lines(plt.gca())
    plt.title("Raw Velocity with Event Markers")
    plt.xlabel("Time (ms)")
    plt.ylabel("Velocity (m/s)")
    plt.legend()
    plt.tight_layout()
    
    # 8) Drift components
    plt.figure()
    plt.plot(t_traj, drift_x, label="drift_x")
    plt.plot(t_traj, drift_y, label="drift_y")
    plt.plot(t_traj, drift_z, label="drift_z")
    draw_event_lines(plt.gca())
    plt.title("Drift Components with Event Markers")
    plt.xlabel("Time (ms)")
    plt.ylabel("Drift (m/s)")
    plt.legend()
    plt.tight_layout()

plt.show()
