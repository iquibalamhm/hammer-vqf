# FPA Tuning Notes

## Problem
Large FPA variations during straight-line walking:
```
📈 FPA = -8.66 deg
📈 FPA = -175.11 deg   ← jump of ~166°
📈 FPA = -136.64 deg
📈 FPA = -0.41 deg
📈 FPA = -116.20 deg
```

## Root Cause
**Overly sensitive rest/motion detection thresholds** were causing:
1. False foot-flat detections during actual walking motion
2. Poor gravity averaging (contaminated with motion samples)
3. Unreliable stride segmentation leading to wildly varying displacement vectors
4. Erroneous heading calculations causing FPA jumps

## Solutions Implemented

### 1. Balanced Detection Thresholds (FINAL TUNED VALUES)
**Original (too sensitive):**
```cpp
FPA_ACC_DIFF_TH = 0.25f   // caused false detections
FPA_GYR_NORM_TH = 0.7f    // ~40 deg/s (too low)
FPA_HYS_FRAC = 0.2f
FPA_MIN_REST_S = 0.05f    // 50ms (too short)
FPA_MIN_MOTION_S = 0.05f
```

**First attempt (too strict - missed most strides):**
```cpp
FPA_ACC_DIFF_TH = 0.5f
FPA_GYR_NORM_TH = 1.5f    // ~86 deg/s (too high)
FPA_MIN_REST_S = 0.15f    // 150ms (too long)
```

**✅ FINAL BALANCED (current):**
```cpp
FPA_ACC_DIFF_TH = 0.35f   // moderate: 0.35 m/s² deviation from gravity
FPA_GYR_NORM_TH = 1.0f    // ~57 deg/s (balanced for normal walking)
FPA_HYS_FRAC = 0.25f      // 25% hysteresis
FPA_MIN_REST_S = 0.08f    // 80ms minimum rest duration
FPA_MIN_MOTION_S = 0.06f  // 60ms minimum motion duration
```

**Why these values work?**
- `0.35 m/s²` accel threshold: tight enough to avoid swing-phase false positives, loose enough to catch real foot-flat
- `1.0 rad/s` gyro threshold: typical mid-stance has < 30°/s rotation; swing has 50-150°/s
- `80ms` rest: typical foot-flat contact is 150-300ms; 80ms catches the stable middle portion
- `60ms` motion: avoids transient noise spikes triggering false motion states

### 2. Moving Average Filter (5-tap)
Added smoothing to reduce stride-to-stride noise:
```cpp
float filterFpa(float rawFpa) {
  // Circular buffer of last 5 FPA values
  // Returns average to smooth outliers
}
```

### 3. Relaxed Stride Validation
Accept most strides; filter only extreme outliers:
```cpp
// Duration: 0.2s to 3.0s (allows slow/fast walking)
validStride = (stride_duration >= 0.2 && stride_duration <= 3.0);

// Displacement: 2cm to 3m (very permissive)
displacement = sqrt(dx² + dy²);
validStride = validStride && (displacement >= 0.02 && displacement <= 3.0);
```

**Note:** The moving average filter handles noisy values; validation only rejects physically impossible strides.

### 4. Enhanced Telemetry
Now outputs raw, filtered, and validation status:
```
,FPA_deg=-8.66,FPA_filt=-7.23,valid=1,...
```
- `FPA_deg`: raw value from current stride
- `FPA_filt`: 5-stride moving average (sent to TCP)
- `valid`: 0 if rejected, 1 if passed validation

## Expected Behavior After Tuning

### Good Walking (Straight Line)
```
📈 FPA = -2.3 deg
📈 FPA = -1.8 deg
📈 FPA = -2.1 deg
📈 FPA = -2.5 deg
```
✅ Variation should be < ±5° for consistent straight-line gait

### Debug Output
Watch for:
- `state=0` during true foot-flat (standing still, mid-stance)
- `state=1` during swing/toe-off
- `grav_count` should be 30-80 samples for 150ms rest @ 200Hz feed rate
- `stride=N` incrementing only at true heel-strike events

## Troubleshooting

### Still seeing large jumps (>30°)?
1. **Check sensor mounting**: ensure IMU is rigidly attached to shoe
2. **Verify axis alignment**: `is_left_foot` flag must match actual foot
3. **Increase thresholds further**: try `ACC_DIFF_TH=0.7f`, `GYR_NORM_TH=2.0f`
4. **Inspect debug output**: look for premature state transitions

### FPA always reads 0° or constant?
1. **Not detecting strides**: thresholds too strict, foot never enters motion state
2. **Reduce thresholds slightly**: try `ACC_DIFF_TH=0.4f`, `GYR_NORM_TH=1.2f`
3. **Check `stride=N` counter**: should increment every 2-3 seconds during walking

### Filtered value lags behind reality?
- Reduce `FPA_FILTER_SIZE` from 5 to 3 taps for faster response
- Trade-off: less smoothing but quicker adaptation

## Testing Protocol
1. **Stand still** for 5 seconds → should see no FPA output (no strides)
2. **Walk straight** 10 steps → FPA should stay within ±5° of mean
3. **Turn corner** gradually → FPA should smoothly transition
4. **Compare debug**: watch Serial output for `state` transitions matching gait cycle

## References
- Laidig & Jocham (2023): "Estimating gait parameters from IMUs"
- Typical gait thresholds: Sabatini et al. (2005), Zijlstra & Hof (2003)
