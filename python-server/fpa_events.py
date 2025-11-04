# fpa_events.py
# Host-side Foot-Flat detection (sections 3.1–3.4) for FC/HR/REST events.

import math
from collections import deque

G_DEFAULT = 9.81

def ridler_calvard_threshold(values, w=0.5, iters=100, t_init=None, min_th=0.0):
    if not values:
        return max(min_th, 0.0)
    if t_init is None:
        t = sum(values)/len(values)
    else:
        t = t_init
    t = max(t, min_th)
    for _ in range(iters):
        low = [v for v in values if v <= t]
        high = [v for v in values if v > t]
        if not low or not high:
            break
        mu_low = sum(low)/len(low)
        mu_high = sum(high)/len(high)
        t_new = w*mu_high + (1.0-w)*mu_low
        if abs(t_new - t) < 1e-6:
            t = t_new
            break
        t = t_new
    return max(t, min_th)


class Schmitt:
    def __init__(self, t_center: float, frac: float, init_state=1):
        self.set_thresholds(t_center, frac)
        self.state = init_state
    
    def set_thresholds(self, t_center: float, frac: float):
        frac = max(0.0, min(0.49, frac))
        self.t_high = t_center * (1.0 + frac)
        self.t_low  = t_center * (1.0 - frac)
        if self.t_low > self.t_high:
            self.t_low, self.t_high = self.t_high, self.t_low
    
    def step(self, value: float) -> int:
        s = self.state
        if s == 0:
            if value >= self.t_high:
                s = 1
        else:
            if value <= self.t_low:
                s = 0
        self.state = s
        return s


def cleanup_short_runs(times, bits, t0_min, t1_min):
    n = len(bits)
    if n == 0:
        return []
    out = bits[:]
    i = 0
    while i < n:
        j = i
        val = out[i]
        while j < n and out[j] == val:
            j += 1
        dt = times[j-1] - times[i]
        if val == 1 and dt < t1_min:
            for k in range(i, j):
                out[k] = 0
        i = j
    i = 0
    while i < n:
        j = i
        val = out[i]
        while j < n and out[j] == val:
            j += 1
        dt = times[j-1] - times[i]
        if val == 0 and dt < t0_min:
            for k in range(i, j):
                out[k] = 1
        i = j
    return out


class FootFlatDetector:
    def __init__(self, 
                 g=G_DEFAULT,
                 win_thresh_s=5.0,
                 hyst_frac=0.20,
                 a_th_min=0.15,
                 w_th_min=0.35,
                 t0_min=0.040,
                 t1_min=0.040,
                 rc_iters=100,
                 w_acc=0.5, w_gyr=0.5):
        self.g = g
        self.win_thresh_s = win_thresh_s
        self.hyst_frac = hyst_frac
        self.a_th_min = a_th_min
        self.w_th_min = w_th_min
        self.t0_min = t0_min
        self.t1_min = t1_min
        self.rc_iters = rc_iters
        self.w_acc = w_acc
        self.w_gyr = w_gyr

        self.t_ms_buf = deque()
        self.da_buf = deque()
        self.gn_buf = deque()
        self.ra_bits = deque()
        self.rw_bits = deque()

        self.sch_a = None
        self.sch_w = None

        self.last_fc_ms = None
        self.last_emitted_fc_ms = None
        self.last_emitted_hr_ms = None
        self.last_emitted_rest_ms = None

    def _maybe_drop_old(self, t_ms_now):
        t_cut = t_ms_now - 1000.0 * max(self.win_thresh_s, (self.t0_min + self.t1_min + 0.5))
        while self.t_ms_buf and self.t_ms_buf[0] < t_cut:
            self.t_ms_buf.popleft()
            self.da_buf.popleft()
            self.gn_buf.popleft()
            if self.ra_bits: self.ra_bits.popleft()
            if self.rw_bits: self.rw_bits.popleft()

    def _compute_thresholds(self):
        vals_a = list(self.da_buf)
        vals_w = list(self.gn_buf)
        if not vals_a or not vals_w:
            return None, None
        ta = ridler_calvard_threshold(vals_a, w=self.w_acc, iters=self.rc_iters, min_th=self.a_th_min)
        tw = ridler_calvard_threshold(vals_w, w=self.w_gyr, iters=self.rc_iters, min_th=self.w_th_min)
        return ta, tw

    def _ensure_schmitt(self, ta, tw):
        if self.sch_a is None:
            self.sch_a = Schmitt(ta, self.hyst_frac, init_state=1)
        else:
            self.sch_a.set_thresholds(ta, self.hyst_frac)
        if self.sch_w is None:
            self.sch_w = Schmitt(tw, self.hyst_frac, init_state=1)
        else:
            self.sch_w.set_thresholds(tw, self.hyst_frac)

    def _compute_clean_r(self):
        times = [t/1000.0 for t in self.t_ms_buf]
        r_or = [1 if (a or b) else 0 for a, b in zip(self.ra_bits, self.rw_bits)]
        r_clean = cleanup_short_runs(times, r_or, self.t0_min, self.t1_min)
        return r_clean

    def update(self, t_ms, ax, ay, az, gx_rad, gy_rad, gz_rad):
        an = math.sqrt(ax*ax + ay*ay + az*az)
        da = abs(an - self.g)
        gn = math.sqrt(gx_rad*gx_rad + gy_rad*gy_rad + gz_rad*gz_rad)

        self.t_ms_buf.append(t_ms)
        self.da_buf.append(da)
        self.gn_buf.append(gn)
        self._maybe_drop_old(t_ms)

        ta, tw = self._compute_thresholds()
        if ta is None or tw is None:
            return []

        self._ensure_schmitt(ta, tw)
        ra = self.sch_a.step(da)
        rw = self.sch_w.step(gn)
        self.ra_bits.append(ra)
        self.rw_bits.append(rw)

        r_clean = self._compute_clean_r()
        if len(r_clean) < 2:
            return []

        r_prev, r_curr = r_clean[-2], r_clean[-1]
        out = []
        if r_prev == 1 and r_curr == 0:
            if self.last_emitted_fc_ms != t_ms:
                out.append(("FC", t_ms))
                self.last_emitted_fc_ms = t_ms
            self.last_fc_ms = t_ms
        elif r_prev == 0 and r_curr == 1:
            if self.last_emitted_hr_ms != t_ms:
                out.append(("HR", t_ms))
                self.last_emitted_hr_ms = t_ms
            if self.last_fc_ms is not None:
                t_rest = 0.5*(self.last_fc_ms + t_ms)
                if self.last_emitted_rest_ms != t_rest:
                    out.append(("REST", t_rest))
                    self.last_emitted_rest_ms = t_rest
        return out
