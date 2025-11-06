#!/usr/bin/env python3
"""
Visualize HR/FC detection results from stridelog.csv
"""
import matplotlib.pyplot as plt
import numpy as np
from utils.hrfc import HRFCDetector

# Load data
filepath = '/home/iquibalh/Documents/cmu/research/HAMMER/hammer-vqf/imu_vqf_stream/stridelog.csv'
data = []
with open(filepath, 'r') as f:
    for line in f:
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split(',')
        if len(parts) != 11:
            continue
        try:
            vals = [float(p) for p in parts]
            data.append(vals)
        except ValueError:
            continue

print(f"Loaded {len(data)} samples")

# Create detector
detector = HRFCDetector(
    g=9.81, th_win=5.0, rc_iters=100, hyst_frac=0.23,
    a_th_min=1.8, w_th_min=0.0, T0_min=0.120, T1_min=0.180,
    w_acc=0.85, w_gyr=0.80, gyro_units='rad'
)

# Process and collect data
t_list, a_err_list, a_th_list, w_norm_list, w_th_list, r_list = [], [], [], [], [], []
hr_events, fc_events = [], []

for row in data:
    t_ms, ax, ay, az, gx, gy, gz, qw, qx, qy, qz = row
    detector.update(t_ms, ax, ay, az, gx, gy, gz, qw, qx, qy, qz)
    
    dbg = detector.get_debug()
    t_list.append(dbg['t'])
    a_err_list.append(dbg['a_err'])
    a_th_list.append(dbg['a_th'])
    w_norm_list.append(dbg['w_norm'])
    w_th_list.append(dbg['w_th'])
    r_list.append(dbg['r'])
    
    for tag, te in detector.pop_events():
        if tag == "HR":
            hr_events.append(te)
        else:
            fc_events.append(te)

print(f"Detected {len(hr_events)} HR events and {len(fc_events)} FC events")

# Plot
fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

# Acceleration error
axes[0].plot(t_list, a_err_list, 'b-', label='|a|-g', linewidth=1.5)
axes[0].plot(t_list, a_th_list, 'r--', label='a_th', linewidth=2)
axes[0].set_ylabel('Accel Error (m/s²)')
axes[0].legend(loc='upper right')
axes[0].grid(True, alpha=0.3)
axes[0].set_title('HR/FC Detection Analysis')

# Gyro norm
axes[1].plot(t_list, w_norm_list, 'g-', label='‖ω‖', linewidth=1.5)
axes[1].plot(t_list, w_th_list, 'orange', linestyle='--', label='w_th', linewidth=2)
axes[1].set_ylabel('Gyro Norm (rad/s)')
axes[1].legend(loc='upper right')
axes[1].grid(True, alpha=0.3)

# Rest state + events
axes[2].plot(t_list, r_list, 'gray', linewidth=2, label='rest r(t)')
axes[2].set_ylabel('State')
axes[2].set_xlabel('Time (s)')
axes[2].set_yticks([0, 1])
axes[2].set_yticklabels(['Motion', 'Rest'])
axes[2].grid(True, alpha=0.3)

# Add event markers
for hr_t in hr_events:
    for ax in axes:
        ax.axvline(hr_t, color='red', linestyle='-', linewidth=2, alpha=0.7)
    axes[2].plot(hr_t, 0, 'rv', markersize=12, label='HR' if hr_t == hr_events[0] else '')

for fc_t in fc_events:
    for ax in axes:
        ax.axvline(fc_t, color='green', linestyle='-', linewidth=2, alpha=0.7)
    axes[2].plot(fc_t, 1, 'g^', markersize=12, label='FC' if fc_t == fc_events[0] else '')

axes[2].legend(loc='upper right')

plt.tight_layout()
plt.savefig('/home/iquibalh/Documents/cmu/research/HAMMER/hammer-vqf/python-server/hrfc_analysis.png', dpi=150)
print("Saved plot to hrfc_analysis.png")
plt.show()
