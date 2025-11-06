"""
Heel Rise (HR) and Full Contact (FC) detection module.
No GUI dependencies - pure logic for adaptive threshold-based gait event detection.
"""
import math
from collections import deque


class RCThreshold:
    """
    Weighted iterative threshold estimation over a sliding time window.
    Uses percentile-based convergence with configurable weights.
    """
    def __init__(self, win_s, rc_iters, w_acc, w_gyr, min_thresh):
        self.win_s = win_s
        self.rc_iters = rc_iters
        self.w_acc = w_acc
        self.w_gyr = w_gyr
        self.min_thresh = min_thresh
        self.samples = deque()  # (t_ms, a_err, w_norm)
    
    def add_sample(self, t_ms, a_err, w_norm):
        """Add a new sample and trim to window."""
        self.samples.append((t_ms, a_err, w_norm))
        t_cut = t_ms - (self.win_s * 1000.0)
        while self.samples and self.samples[0][0] < t_cut:
            self.samples.popleft()
    
    def compute_threshold(self):
        """
        Compute adaptive thresholds using iterative percentile method.
        Returns (a_th, w_th).
        """
        if not self.samples:
            return self.min_thresh, self.min_thresh
        
        # Extract a_err and w_norm arrays
        a_vals = [a for (_, a, _) in self.samples]
        w_vals = [w for (_, _, w) in self.samples]
        
        if not a_vals or not w_vals:
            return self.min_thresh, self.min_thresh
        
        # Iterative convergence: start at median, refine by taking percentile of values below threshold
        a_th = sorted(a_vals)[len(a_vals) // 2]
        w_th = sorted(w_vals)[len(w_vals) // 2]
        
        for _ in range(self.rc_iters):
            # Accel threshold: percentile of values below current threshold
            a_below = [a for a in a_vals if a < a_th]
            if a_below:
                a_below_sorted = sorted(a_below)
                idx = int(len(a_below_sorted) * 0.75)  # 75th percentile
                a_th = a_below_sorted[min(idx, len(a_below_sorted) - 1)]
            
            # Gyro threshold: percentile of values below current threshold
            w_below = [w for w in w_vals if w < w_th]
            if w_below:
                w_below_sorted = sorted(w_below)
                idx = int(len(w_below_sorted) * 0.75)
                w_th = w_below_sorted[min(idx, len(w_below_sorted) - 1)]
        
        # Apply minimum thresholds
        a_th = max(a_th, self.min_thresh)
        w_th = max(w_th, self.min_thresh)
        
        return a_th, w_th


class HysteresisRest:
    """
    Schmitt trigger with hysteresis for rest detection.
    Combines accel and gyro signals with configurable weights.
    """
    def __init__(self, hyst_frac, w_acc, w_gyr):
        self.hyst_frac = hyst_frac
        self.w_acc = w_acc
        self.w_gyr = w_gyr
        self.state = 0  # 0 = moving, 1 = rest
    
    def update(self, a_err, a_th, w_norm, w_th):
        """
        Update state based on weighted combination of accel and gyro.
        Returns current state (0 or 1).
        """
        # Normalize signals by thresholds
        a_norm = a_err / a_th if a_th > 0 else 0.0
        w_norm_scaled = w_norm / w_th if w_th > 0 else 0.0
        
        # Weighted combination
        combined = self.w_acc * a_norm + self.w_gyr * w_norm_scaled
        
        # Schmitt trigger with hysteresis
        if self.state == 0:  # Currently moving
            if combined < (1.0 - self.hyst_frac):
                self.state = 1  # Transition to rest
        else:  # Currently rest
            if combined > (1.0 + self.hyst_frac):
                self.state = 0  # Transition to moving
        
        return self.state


class DwellConfirm:
    """
    Dwell-time confirmation for state transitions.
    Emits HR on confirmed 1→0 and FC on confirmed 0→1.
    """
    def __init__(self, T0_min, T1_min):
        self.T0_min = T0_min * 1000.0  # Convert to ms
        self.T1_min = T1_min * 1000.0
        self.current_state = None
        self.state_start_t = None
        self.pending_events = []
    
    def update(self, t_ms, state):
        """
        Update with new state. Returns list of confirmed events [(tag, t_ms), ...].
        """
        events = []
        
        if self.current_state is None:
            self.current_state = state
            self.state_start_t = t_ms
            return events
        
        if state != self.current_state:
            # State change detected
            duration = t_ms - self.state_start_t
            
            if self.current_state == 1:  # Was rest, now moving (1→0)
                # Confirm as HR if rest duration >= T1_min
                if duration >= self.T1_min:
                    events.append(("HR", t_ms))
            else:  # Was moving, now rest (0→1)
                # Confirm as FC if movement duration >= T0_min
                if duration >= self.T0_min:
                    events.append(("FC", t_ms))
            
            self.current_state = state
            self.state_start_t = t_ms
        
        return events


class HRFCDetector:
    """
    Main detector combining threshold estimation, hysteresis, and dwell confirmation.
    """
    def __init__(self, g, th_win, rc_iters, hyst_frac, 
                 a_th_min, w_th_min, w_acc, w_gyr,
                 T0_min, T1_min, gyro_units="rad"):
        self.g = g
        self.gyro_units = gyro_units
        
        # Components
        self.rc_accel = RCThreshold(th_win, rc_iters, w_acc, w_gyr, a_th_min)
        self.rc_gyro = RCThreshold(th_win, rc_iters, w_acc, w_gyr, w_th_min)
        self.hysteresis = HysteresisRest(hyst_frac, w_acc, w_gyr)
        self.dwell = DwellConfirm(T0_min, T1_min)
        
        # Debug state
        self.last_debug = {
            't': 0.0,
            'a_err': 0.0,
            'a_th': a_th_min,
            'w_norm': 0.0,
            'w_th': w_th_min,
            'r': 0
        }
        
        # Event buffer
        self.event_buffer = []
    
    def update(self, t_ms, ax, ay, az, gx, gy, gz, qw=None, qx=None, qy=None, qz=None):
        """
        Process new IMU sample. Quaternion args are optional (not used in current logic).
        """
        # Compute accel error: |‖a‖ - g|
        a_norm = math.sqrt(ax*ax + ay*ay + az*az)
        a_err = abs(a_norm - self.g)
        
        # Compute gyro norm (ensure it's in rad/s)
        if self.gyro_units == "deg":
            gx_rad = math.radians(gx)
            gy_rad = math.radians(gy)
            gz_rad = math.radians(gz)
        else:
            gx_rad = gx
            gy_rad = gy
            gz_rad = gz
        
        w_norm = math.sqrt(gx_rad*gx_rad + gy_rad*gy_rad + gz_rad*gz_rad)
        
        # Add samples to threshold estimators
        self.rc_accel.add_sample(t_ms, a_err, w_norm)
        self.rc_gyro.add_sample(t_ms, a_err, w_norm)
        
        # Compute adaptive thresholds
        a_th, _ = self.rc_accel.compute_threshold()
        _, w_th = self.rc_gyro.compute_threshold()
        
        # Update hysteresis state
        r = self.hysteresis.update(a_err, a_th, w_norm, w_th)
        
        # Check for dwell-confirmed events
        events = self.dwell.update(t_ms, r)
        self.event_buffer.extend(events)
        
        # Store debug info
        self.last_debug = {
            't': t_ms / 1000.0,
            'a_err': a_err,
            'a_th': a_th,
            'w_norm': w_norm,
            'w_th': w_th,
            'r': r
        }
    
    def pop_events(self):
        """
        Retrieve and clear pending events.
        Returns [(tag, t_s), ...] where t_s is time in seconds.
        """
        events = [(tag, t_ms / 1000.0) for tag, t_ms in self.event_buffer]
        self.event_buffer.clear()
        return events
    
    def get_debug(self):
        """
        Get current debug state for visualization.
        Returns dict with keys: 't', 'a_err', 'a_th', 'w_norm', 'w_th', 'r'
        """
        return self.last_debug.copy()
