# HR/FC Detection Implementation

## Overview
This implementation provides real-time Heel Rise (HR) and Full Contact (FC) detection for gait analysis using IMU data.

## Architecture

### Core Module: `utils/hrfc.py`
Contains the detection logic with no GUI dependencies:

- **RCThreshold**: Adaptive threshold estimation using iterative percentile method over sliding window
- **HysteresisRest**: Schmitt trigger for robust rest/motion state detection
- **DwellConfirm**: Dwell-time confirmation to prevent spurious events
- **HRFCDetector**: Main detector that orchestrates all components

### Visualization: `utils/plotter.py`
Enhanced IMUWindow with:
- Accel panel overlays: `|a|-g` and adaptive threshold `a_th`
- Gyro panel overlays: `‖ω‖` and adaptive threshold `w_th`
- Event strip panel: Binary rest signal `r(t)` with HR/FC event markers

## Usage

### Basic Command
```bash
python imu_fc_hr_host.py --port /dev/ttyACM0 --baud 115200 --gyro-units rad --rpy-3d
```

### With Custom Parameters
```bash
python imu_fc_hr_host.py --port /dev/ttyACM0 --baud 115200 --gyro-units rad \
  --hyst-frac 0.23 \
  --a-th-min 1.8 \
  --w-th-min 0.0 \
  --t0-min 0.120 \
  --t1-min 0.180 \
  --w-acc 0.85 \
  --w-gyr 0.80 \
  --th-win 5.0 \
  --rpy-3d
```

### From File
```bash
python imu_fc_hr_host.py --file sample.csv --gyro-units rad
```

## Parameters (Research-Backed Defaults)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--g` | 9.81 | Gravity constant (m/s²) |
| `--th-win` | 5.0 | Threshold estimation window (seconds) |
| `--rc-iters` | 100 | Iterative threshold iterations |
| `--hyst-frac` | 0.23 | Schmitt hysteresis fraction |
| `--a-th-min` | 1.8 | Minimum accel threshold (m/s²) |
| `--w-th-min` | 0.0 | Minimum gyro threshold (rad/s) |
| `--t0-min` | 0.120 | Minimum movement duration (sec) |
| `--t1-min` | 0.180 | Minimum rest duration (sec) |
| `--w-acc` | 0.85 | Weight for accel in combined signal |
| `--w-gyr` | 0.80 | Weight for gyro in combined signal |

## Visual Output

### Plot Layout (Top to Bottom)
1. **Acceleration**: ax, ay, az + `|a|-g` (yellow) + `a_th` (dashed orange)
2. **Gyroscope**: gx, gy, gz + `‖ω‖` (yellow) + `w_th` (dashed orange)
3. **Quaternion**: qw, qx, qy, qz (raw from device)
4. **Orientation**: roll, pitch, yaw (converted from quaternion)
5. **Event Strip**: 
   - Gray step signal: rest state (0=moving, 1=rest)
   - Red triangles (down) at y=0: HR events
   - Green triangles (up) at y=1: FC events

### Console Output
Events are printed as CSV:
```
EVT,<timestamp_seconds>,HR
EVT,<timestamp_seconds>,FC
```

## Expected Input Format
CSV lines with 11 values:
```
t_ms,ax,ay,az,gx,gy,gz,qw,qx,qy,qz
```

Where:
- `t_ms`: timestamp in milliseconds
- `ax,ay,az`: acceleration (m/s²)
- `gx,gy,gz`: gyroscope (rad/s or deg/s, specify with `--gyro-units`)
- `qw,qx,qy,qz`: quaternion from device

## Algorithm Summary

1. **Compute Error Signals**:
   - `a_err = |‖a‖ - g|`
   - `w_norm = ‖ω‖`

2. **Adaptive Thresholds**:
   - Iterative percentile method over sliding window
   - Converges to 75th percentile of sub-threshold values

3. **Rest Detection**:
   - Weighted combination: `combined = w_acc × (a_err/a_th) + w_gyr × (w_norm/w_th)`
   - Schmitt trigger with hysteresis

4. **Event Confirmation**:
   - HR: Rest→Motion transition after ≥ t1_min rest
   - FC: Motion→Rest transition after ≥ t0_min motion

## Verification Checklist

- [ ] Rolling 5s window shows `|a|-g` and `‖ω‖` with moving thresholds
- [ ] Bottom strip shows binary rest signal
- [ ] HR markers appear at y=0 when foot lifts
- [ ] FC markers appear at y=1 when foot contacts
- [ ] Console prints `EVT,<t>,HR/FC` as events occur
- [ ] Smooth refresh at UI update rate (default 60 Hz)

## Notes

- Events may not appear in perfect real-time due to dwell confirmation
- The 5-second threshold window needs warm-up time
- Adjust parameters based on gait speed and pathology
- For batch processing, events are stored and can be exported
