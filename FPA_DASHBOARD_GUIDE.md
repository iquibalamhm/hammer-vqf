# FPA Dashboard - Core Detection Tuning Guide

## Overview
The FPA Dashboard now includes controls to tune the **core stride detection thresholds** directly from your phone, without reflashing the firmware.

## Web Interface

### Access
1. Start the dashboard server on your laptop:
   ```bash
   python esp_fpa_dashboard.py --bind 0.0.0.0 --http 8000 --tcp-port 12345
   ```

2. Upload `main_web.cpp` firmware to your ESP32 (not `main.cpp`)

3. Open on your phone (same Wi-Fi):
   ```
   http://<laptop-ip>:8000/
   ```

## New Controls: Core Detection Thresholds

### Parameters

**Accel diff threshold (m/s²)** - Default: `0.35`
- How much acceleration can deviate from gravity (9.81 m/s²) during rest
- **Lower** = more sensitive (detects rest even with small movements)
- **Higher** = stricter (requires very still foot for rest detection)
- Range: `0.2` - `0.7`

**Gyro norm threshold (rad/s)** - Default: `1.0` (~57°/s)
- Maximum gyroscope magnitude during rest
- **Lower** = more sensitive (detects rest with slower rotation)
- **Higher** = stricter (requires very still rotation)
- Range: `0.5` - `2.0`

**Hysteresis fraction** - Default: `0.25` (25%)
- Prevents rapid switching between rest/motion states
- **Lower** = faster transitions (may flutter)
- **Higher** = slower transitions (more stable)
- Range: `0.1` - `0.4`

**Min rest duration (s)** - Default: `0.08` (80ms)
- Minimum time foot must be still to count as rest
- **Lower** = detects brief touches
- **Higher** = only counts sustained contact
- Range: `0.05` - `0.15`

**Min motion duration (s)** - Default: `0.06` (60ms)
- Minimum time foot must be moving to count as motion
- **Lower** = detects quick movements
- **Higher** = filters transients
- Range: `0.04` - `0.12`

## Tuning Workflow

### Problem: Missing strides (only 1 in 4 steps detected)
**Solution:** Make detection MORE sensitive
```
Accel diff: 0.35 → 0.30
Gyro norm:  1.0  → 0.85
Min rest:   0.08 → 0.06
```

### Problem: False strides (detects during swing phase)
**Solution:** Make detection LESS sensitive (stricter)
```
Accel diff: 0.35 → 0.45
Gyro norm:  1.0  → 1.3
Min rest:   0.08 → 0.10
```

### Problem: Wild FPA variations (-8° → -175° → -136°)
**Solution:** Increase hysteresis and minimum durations
```
Hysteresis: 0.25 → 0.30
Min rest:   0.08 → 0.12
Min motion: 0.06 → 0.08
```

## Quick Actions

### Reset to Defaults
Click **"Reset to Defaults"** button to restore balanced values:
- `acc_diff_th = 0.35`
- `gyr_norm_th = 1.0`
- `hys_frac = 0.25`
- `min_rest_s = 0.08`
- `min_motion_s = 0.06`

### Live Tuning
1. Enter new values in the input fields
2. Click **"Apply Core Settings"**
3. Walk and observe FPA output
4. Adjust and repeat until satisfied

## ESP32 Commands (Alternative)

You can also send commands directly via TCP:

```
fpa.acc_diff_th=0.30
fpa.gyr_norm_th=0.85
fpa.hys_frac=0.25
fpa.min_rest_s=0.06
fpa.min_motion_s=0.06
```

The firmware will acknowledge with:
```
ACK fpa.acc_diff_th=0.30
```

## Tips

1. **Start conservative** (default values), then adjust one parameter at a time
2. **Test while walking** - observe stride detection rate in real-time
3. **Check Serial output** - look for `valid=0` markers (rejected strides)
4. **Surface matters** - carpet vs hard floor may need different settings
5. **Walking speed** - slower walking needs lower thresholds

## Monitoring

Watch the dashboard for:
- **FPA plot smoothness** - should be consistent during straight walking
- **Connection status** - green = good, amber = Wi-Fi only, red = disconnected
- **Update frequency** - should match your step rate (0.5-2 Hz)

## Troubleshooting

**Dashboard shows "Disconnected"**
- Check ESP32 is on same Wi-Fi network
- Verify TCP_SERVER_IP matches your laptop IP
- Look for green LED on ESP32 (connected)

**Changes don't apply**
- Ensure you uploaded `main_web.cpp` firmware (not `main.cpp`)
- Check Serial monitor for "ACK" responses
- Restart ESP32 if unresponsive

**FPA still unstable**
- Try increasing median filter window: `median_win=50`
- Lower lowpass cutoff: `lowpass_hz=2.0`
- These are in the "FPA Parameters" section (top card)
