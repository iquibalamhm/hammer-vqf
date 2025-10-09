#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32 FPA Dashboard (TCP bridge + mobile web UI)

- Keeps your existing TCP link to the ESP32 (board connects as a TCP client).
- Serves a tiny web app you can open from your phone to see FPA and tweak params.
- Lets you reset ("zero") the IMU orientation from the phone.

Run:
  python esp_fpa_dashboard.py --bind 0.0.0.0 --http 8000 --tcp-port 12345

Then on your phone (same Wi‑Fi):
  http://<your-laptop-ip>:8000/

Protocol to the ESP32:
  * Telemetry lines (any order):
      footRoll_deg=...,footPitch_deg=...,footYaw_deg=...
      fpa_deg=...
  * Commands (one per line):
      ze                      -> zero/align IMU yaw (keep gravity)
      fpa.offset_deg=FLOAT    -> static offset added to FPA
      fpa.median_win=INT      -> median filter window (samples)
      fpa.lowpass_hz=FLOAT    -> one‑pole LPF cutoff (Hz)
    vqf.tau_acc=FLOAT       -> accelerometer fusion time constant (s)
    vqf.tau_mag=FLOAT       -> magnetometer fusion time constant (s)
    vqf.motion_bias=0|1     -> enable motion bias estimator
    vqf.rest_bias=0|1       -> enable rest bias estimator
    vqf.mag_reject=0|1      -> enable mag disturbance rejection
    vqf.use_mag=0|1         -> feed magnetometer to VQF
    vqf.reset               -> reinitialize VQF state
You can rename/tweak keys if you prefer; just mirror on firmware.
"""
import argparse
import json
import socket
import threading
import time
import re
from typing import Optional, Dict, Any

from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, JSONResponse
import uvicorn

FOOT_RE = re.compile(
    r"footRoll_deg=([-+]?\d*\.?\d+|nan),"
    r"footPitch_deg=([-+]?\d*\.?\d+|nan),"
    r"footYaw_deg=([-+]?\d*\.?\d+|nan)",
    re.IGNORECASE,
)
FPA_RE = re.compile(r"fpa_deg=([-+]?\d*\.?\d+)", re.IGNORECASE)
QUAT_RE = re.compile(
    r"quat_w=([-+]?\d*\.?\d+),quat_x=([-+]?\d*\.?\d+),"
    r"quat_y=([-+]?\d*\.?\d+),quat_z=([-+]?\d*\.?\d+)",
    re.IGNORECASE,
)


def _to_float(tok: str) -> float:
    try:
        return float(tok)
    except Exception:
        return float("nan")


class SharedState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.connected = False
        self.last_seen = 0.0
        self.foot_rpy = [float("nan")] * 3
        self.fpa_deg = float("nan")
        self.quat = [float("nan")] * 4

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            # Convert NaN to None for JSON compliance
            def safe_float(x: float) -> Optional[float]:
                import math
                return None if math.isnan(x) or math.isinf(x) else x
            
            return {
                "connected": self.connected,
                "last_seen_s_ago": max(0.0, time.time() - self.last_seen) if self.last_seen else None,
                "foot_roll_deg": safe_float(self.foot_rpy[0]),
                "foot_pitch_deg": safe_float(self.foot_rpy[1]),
                "foot_yaw_deg": safe_float(self.foot_rpy[2]),
                "fpa_deg": safe_float(self.fpa_deg),
                "quat_w": safe_float(self.quat[0]),
                "quat_x": safe_float(self.quat[1]),
                "quat_y": safe_float(self.quat[2]),
                "quat_z": safe_float(self.quat[3]),
            }


class TcpBridge(threading.Thread):
    def __init__(self, host: str, port: int, state: SharedState, verbose: bool = True):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.state = state
        self.verbose = verbose
        self._stop_evt = threading.Event()
        self._conn_lock = threading.Lock()
        self._conn: Optional[socket.socket] = None

    # ---------- command path ----------
    def send_line(self, line: str) -> bool:
        line = (line or "").strip()
        if not line:
            return False
        with self._conn_lock:
            if not self._conn:
                return False
            try:
                self._conn.sendall((line + "\n").encode("utf-8"))
                return True
            except OSError:
                return False

    # ---------- telemetry path ----------
    def _handle_text(self, txt: str) -> None:
        m = FOOT_RE.search(txt)
        now = time.time()
        if m:
            r, p, y = (_to_float(t) for t in m.groups())
            with self.state.lock:
                self.state.foot_rpy[0] = r
                self.state.foot_rpy[1] = p
                self.state.foot_rpy[2] = y
                self.state.last_seen = now
            return
        m2 = FPA_RE.search(txt)
        if m2:
            try:
                f = float(m2.group(1))
            except Exception:
                f = float("nan")
            with self.state.lock:
                self.state.fpa_deg = f
                self.state.last_seen = now
            return
        m3 = QUAT_RE.search(txt)
        if m3:
            vals = [_to_float(t) for t in m3.groups()]
            with self.state.lock:
                for i in range(4):
                    self.state.quat[i] = vals[i]
                self.state.last_seen = now
            return
        if self.verbose:
            # Any other info line:
            print("ℹ️ ", txt.strip())

    # ---------- accept loop ----------
    def run(self) -> None:
        print(f"📶 Waiting for ESP32 on {self.host}:{self.port} ...")
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((self.host, self.port))
            srv.listen(1)
            srv.settimeout(1.0)
            while not self._stop_evt.is_set():
                try:
                    conn, addr = srv.accept()
                except socket.timeout:
                    continue
                except OSError as e:
                    print("[accept error]", e)
                    continue
                with self._conn_lock:
                    self._conn = conn
                with self.state.lock:
                    self.state.connected = True
                print(f"✅ ESP32 connected from {addr}")

                try:
                    buf = b""
                    conn.settimeout(1.0)
                    while not self._stop_evt.is_set():
                        try:
                            data = conn.recv(1024)
                        except socket.timeout:
                            continue
                        if not data:
                            break
                        buf += data
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                            try:
                                s = line.decode("utf-8", "ignore")
                            except Exception:
                                s = ""
                            if s:
                                self._handle_text(s)
                except Exception as e:
                    print("[conn error]", e)
                finally:
                    with self._conn_lock:
                        try:
                            conn.close()
                        except Exception:
                            pass
                        self._conn = None
                    with self.state.lock:
                        self.state.connected = False
                    print("ℹ️ ESP32 disconnected")

    def stop(self) -> None:
        self._stop_evt.set()


# ----------------------- HTTP app -----------------------
def make_app(bridge: TcpBridge, state: SharedState) -> FastAPI:
    app = FastAPI()

    @app.get("/", response_class=HTMLResponse)
    async def index() -> str:
        # Minimal, mobile‑friendly UI with polling + controls
        return HTML_PAGE

    @app.get("/api/state")
    async def api_state() -> JSONResponse:
        return JSONResponse(state.snapshot())

    @app.post("/api/cmd")
    async def api_cmd(request: Request) -> JSONResponse:
        try:
            payload = await request.json()
        except Exception:
            payload = {}
        action = (payload.get("action") or "").lower()
        raw = payload.get("raw")
        ok = False
        if raw:
            ok = bridge.send_line(str(raw))
        elif action == "reset_bno" or action == "zero":
            ok = bridge.send_line("ze")
        else:
            # Unknown -> pass through if stringy
            if isinstance(payload, str):
                ok = bridge.send_line(payload)
        return JSONResponse({"ok": ok})

    @app.post("/api/fpa")
    async def api_fpa(request: Request) -> JSONResponse:
        try:
            params = await request.json()
        except Exception:
            params = {}
        sent = {}
        mapping = {
            "offset_deg": "fpa.offset_deg",
            "median_win": "fpa.median_win",
            "lowpass_hz": "fpa.lowpass_hz",
        }
        for k, v in params.items():
            if k in mapping:
                line = f"{mapping[k]}={v}"
                ok = bridge.send_line(line)
                sent[k] = bool(ok)
        return JSONResponse({"sent": sent})

    @app.post("/api/vqf")
    async def api_vqf(request: Request) -> JSONResponse:
        try:
            params = await request.json()
        except Exception:
            params = {}
        sent = {}
        mapping = {
            "tau_acc": "vqf.tau_acc",
            "tau_mag": "vqf.tau_mag",
            "motion_bias": "vqf.motion_bias",
            "rest_bias": "vqf.rest_bias",
            "mag_reject": "vqf.mag_reject",
            "use_mag": "vqf.use_mag",
        }
        for k, v in params.items():
            if k in mapping:
                line = f"{mapping[k]}={v}"
                ok = bridge.send_line(line)
                sent[k] = bool(ok)
        return JSONResponse({"sent": sent})

    @app.post("/api/fpa/core")
    async def api_fpa_core(request: Request) -> JSONResponse:
        """Configure core FPA detection thresholds."""
        try:
            params = await request.json()
        except Exception:
            params = {}
        sent = {}
        mapping = {
            "acc_diff_th": "fpa.acc_diff_th",
            "gyr_norm_th": "fpa.gyr_norm_th",
            "hys_frac": "fpa.hys_frac",
            "min_rest_s": "fpa.min_rest_s",
            "min_motion_s": "fpa.min_motion_s",
        }
        for k, v in params.items():
            if k in mapping:
                line = f"{mapping[k]}={v}"
                ok = bridge.send_line(line)
                sent[k] = bool(ok)
        return JSONResponse({"sent": sent})

    return app


HTML_PAGE = r"""
<!doctype html>
<html lang="en">
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>FPA Dashboard</title>
<style>
  :root { --bg:#0d0f12; --fg:#e7e9ee; --muted:#a7acb8; --card:#141820; --accent:#5ee1a1; --warn:#f9d36f; --bad:#ff7a90; }
  html, body { background:var(--bg); color:var(--fg); font-family: system-ui, -apple-system, Segoe UI, Roboto, Helvetica, Arial, "Apple Color Emoji","Segoe UI Emoji"; margin:0; }
  .wrap { max-width: 750px; margin: 0 auto; padding: 16px; }
  .row { display:flex; gap:12px; flex-wrap:wrap; }
  .card { background:var(--card); border-radius:18px; padding:16px; box-shadow: 0 10px 30px rgba(0,0,0,0.25); flex:1; min-width:260px; }
  h1 { font-size: 22px; margin: 0 0 12px; letter-spacing: 0.2px;}
  h2 { font-size: 14px; margin: 0 0 8px; color: var(--muted); font-weight:600; text-transform: uppercase; letter-spacing:0.08em;}
  .metric { font-size: 40px; font-weight:700; letter-spacing:0.02em; }
  .muted { color: var(--muted); font-size: 13px; }
  .badge { display:inline-block; padding:2px 8px; border-radius:999px; font-size:12px; font-weight:600; }
  .ok { background: rgba(94,225,161,0.15); color: var(--accent); }
  .bad { background: rgba(255,122,144,0.15); color: var(--bad); }
  label { display:block; font-size:13px; color:var(--muted); margin-bottom:4px; }
  input { width:100%; padding:10px 12px; border-radius:12px; border:1px solid #2a3240; background:#0b0e13; color:var(--fg); }
  button { padding:10px 14px; border-radius:12px; border:0; background:#1f2835; color:var(--fg); font-weight:600; }
  button.primary { background: var(--accent); color:#0b1118; }
  .grid { display:grid; grid-template-columns: repeat(2, minmax(0,1fr)); gap:12px; }
  canvas { width:100%; height:120px; display:block; background:#0b0e13; border-radius:12px; }
  .footer { text-align:center; margin-top: 16px; color: var(--muted); font-size:12px;}
</style>
<div class="wrap">
  <h1>Foot Progression Angle (FPA) Dashboard</h1>

  <div class="row">
    <div class="card">
      <h2>ESP32 Link</h2>
      <div id="conn" class="badge bad">Disconnected</div>
      <div class="muted" id="age">Last update: —</div>
      <div style="margin-top:8px">
        <button id="btnZero">Reset / Zero IMU</button>
      </div>
    </div>
    <div class="card">
      <h2>FPA</h2>
      <div class="metric"><span id="fpa">—</span>°</div>
      <canvas id="plot"></canvas>
    </div>
  </div>

    <div class="card" style="margin-top:12px">
        <h2>Foot Orientation</h2>
        <div class="grid" style="grid-template-columns: repeat(auto-fill,minmax(120px,1fr));">
            <div><label>Roll (deg)</label><div class="metric" style="font-size:1.4rem"><span id="roll">—</span></div></div>
            <div><label>Pitch (deg)</label><div class="metric" style="font-size:1.4rem"><span id="pitch">—</span></div></div>
            <div><label>Yaw (deg)</label><div class="metric" style="font-size:1.4rem"><span id="yaw">—</span></div></div>
        </div>
        <div class="grid" style="margin-top:12px; grid-template-columns: repeat(auto-fill,minmax(150px,1fr));">
            <div><label>q<sub>w</sub></label><div class="muted" id="quat_w">—</div></div>
            <div><label>q<sub>x</sub></label><div class="muted" id="quat_x">—</div></div>
            <div><label>q<sub>y</sub></label><div class="muted" id="quat_y">—</div></div>
            <div><label>q<sub>z</sub></label><div class="muted" id="quat_z">—</div></div>
        </div>
    </div>

  <div class="card" style="margin-top:12px">
    <h2>FPA Post-Processing</h2>
    <div class="grid">
      <div><label>Offset (deg)</label><input id="offset" type="number" step="0.1" placeholder="0.0"></div>
      <div><label>Median window (samples)</label><input id="median" type="number" step="1" placeholder="1"></div>
      <div><label>Low-pass cutoff (Hz)</label><input id="lpf" type="number" step="0.1" placeholder="0.0"></div>
    </div>
    <div style="margin-top:10px; display:flex; gap:8px; flex-wrap:wrap;">
      <button class="primary" id="apply">Apply Filters</button>
      <button id="disableFilters">Disable All Filters</button>
      <button id="nudgeL">Nudge −0.5°</button>
      <button id="nudgeR">Nudge +0.5°</button>
    </div>
  </div>

  <div class="card" style="margin-top:12px">
    <h2>Core Detection Thresholds</h2>
    <p class="muted" style="margin-bottom:12px">Adjust stride detection sensitivity. Lower values = more sensitive.</p>
    <div class="grid">
      <div><label>Accel diff threshold (m/s²)</label><input id="acc_diff_th" type="number" step="0.05" placeholder="0.35"></div>
      <div><label>Gyro norm threshold (rad/s)</label><input id="gyr_norm_th" type="number" step="0.1" placeholder="1.0"></div>
      <div><label>Hysteresis fraction</label><input id="hys_frac" type="number" step="0.05" placeholder="0.25"></div>
      <div><label>Min rest duration (s)</label><input id="min_rest_s" type="number" step="0.01" placeholder="0.08"></div>
      <div><label>Min motion duration (s)</label><input id="min_motion_s" type="number" step="0.01" placeholder="0.06"></div>
    </div>
    <div style="margin-top:10px;">
      <button class="primary" id="applyCore">Apply Core Settings</button>
      <button id="resetCore" style="margin-left:8px">Reset to Defaults</button>
    </div>
  </div>

    <div class="card" style="margin-top:12px">
        <h2>VQF Fusion</h2>
        <p class="muted" style="margin-bottom:12px">Tune the orientation filter if you notice drift or jitter.</p>
        <div class="grid">
            <div><label>Tau<sub>acc</sub> (s)</label><input id="tau_acc" type="number" step="0.5" placeholder="3.0"></div>
            <div><label>Tau<sub>mag</sub> (s)</label><input id="tau_mag" type="number" step="0.5" placeholder="9.0"></div>
        </div>
            <div class="grid" style="margin-top:12px; grid-template-columns: repeat(auto-fill,minmax(150px,1fr));">
                <label style="display:flex; align-items:center; gap:6px;"><input type="checkbox" id="motion_bias" checked>Motion bias estimator</label>
                <label style="display:flex; align-items:center; gap:6px;"><input type="checkbox" id="rest_bias" checked>Rest bias estimator</label>
                <label style="display:flex; align-items:center; gap:6px;"><input type="checkbox" id="mag_reject" checked>Mag disturbance reject</label>
                <label style="display:flex; align-items:center; gap:6px;"><input type="checkbox" id="use_mag" checked>Use magnetometer</label>
        </div>
        <div style="margin-top:12px; display:flex; gap:8px; flex-wrap:wrap;">
            <button class="primary" id="applyVqf">Apply VQF Settings</button>
            <button id="resetVqf">Reset VQF</button>
            <button id="vqfDefaults">Defaults</button>
        </div>
    </div>

  <div class="footer">Open this page on your phone while your laptop runs the server on the same Wi‑Fi.</div>
</div>

<script>
const fmt = v => (v===null || v===undefined || Number.isNaN(v)) ? "—" : (+v).toFixed(2);
const fmtQuat = v => (v===null || v===undefined || Number.isNaN(v)) ? "—" : (+v).toFixed(4);
const fpaHistory = [];
const plot = document.getElementById('plot');
const ctx = plot.getContext('2d');

function draw() {
  const w = plot.clientWidth, h = plot.clientHeight;
  if (plot.width !== w) plot.width = w;
  if (plot.height !== h) plot.height = h;
  ctx.clearRect(0,0,w,h);
  ctx.strokeStyle = '#5ee1a1';
  ctx.lineWidth = 2;
  ctx.beginPath();
  const N = fpaHistory.length;
  if (N === 0) return;
  let min = Math.min(...fpaHistory), max = Math.max(...fpaHistory);
  if (min === max) { min -= 1; max += 1; }
  for (let i=0;i<N;i++){
    const x = (i/(N-1))*w;
    const y = h - ((fpaHistory[i]-min)/(max-min))*h;
    if (i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  }
  ctx.stroke();
}

async function poll() {
  try {
    const r = await fetch('/api/state');
    const s = await r.json();
    document.getElementById('conn').textContent = s.connected ? 'Connected' : 'Disconnected';
    document.getElementById('conn').className = 'badge ' + (s.connected ? 'ok':'bad');
    document.getElementById('fpa').textContent = fmt(s.fpa_deg);
    document.getElementById('age').textContent = 'Last update: ' + (s.last_seen_s_ago==null? '—' : s.last_seen_s_ago.toFixed(1)+'s ago');
        document.getElementById('roll').textContent = fmt(s.foot_roll_deg);
        document.getElementById('pitch').textContent = fmt(s.foot_pitch_deg);
        document.getElementById('yaw').textContent = fmt(s.foot_yaw_deg);
        document.getElementById('quat_w').textContent = fmtQuat(s.quat_w);
        document.getElementById('quat_x').textContent = fmtQuat(s.quat_x);
        document.getElementById('quat_y').textContent = fmtQuat(s.quat_y);
        document.getElementById('quat_z').textContent = fmtQuat(s.quat_z);
    if (!Number.isNaN(+s.fpa_deg)){
      fpaHistory.push(+s.fpa_deg);
      if (fpaHistory.length>200) fpaHistory.shift();
      draw();
    }
  } catch(e){/* ignore */}
  setTimeout(poll, 200);
}

async function sendCmd(body){
  await fetch('/api/cmd', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(body)});
}
async function sendFpa(params){
  await fetch('/api/fpa', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(params)});
}
async function sendFpaCore(params){
  await fetch('/api/fpa/core', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(params)});
}
async function sendVqf(params){
    await fetch('/api/vqf', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(params)});
}

document.getElementById('btnZero').onclick = ()=> sendCmd({action:'reset_bno'});
document.getElementById('apply').onclick = ()=>{
  const params = {};
  const off = document.getElementById('offset').value;
  const med = document.getElementById('median').value;
  const lpf = document.getElementById('lpf').value;
  if (off!=='') params.offset_deg = +off;
  if (med!=='') params.median_win = +med;
  if (lpf!=='') params.lowpass_hz = +lpf;
  sendFpa(params);
};
document.getElementById('nudgeL').onclick = ()=>{
  const cur = +(document.getElementById('offset').value || 0);
  document.getElementById('offset').value = (cur - 0.5).toFixed(1);
  sendFpa({offset_deg: cur - 0.5});
};
document.getElementById('nudgeR').onclick = ()=>{
  const cur = +(document.getElementById('offset').value || 0);
  document.getElementById('offset').value = (cur + 0.5).toFixed(1);
  sendFpa({offset_deg: cur + 0.5});
};

document.getElementById('disableFilters').onclick = ()=>{
  // Set values that disable all post-processing
  document.getElementById('offset').value = '0.0';
  document.getElementById('median').value = '1';
  document.getElementById('lpf').value = '0.0';
  sendFpa({
    offset_deg: 0.0,
    median_win: 1,
    lowpass_hz: 0.0
  });
};

document.getElementById('applyVqf').onclick = ()=>{
    const payload = {};
    const tauA = document.getElementById('tau_acc').value;
    const tauM = document.getElementById('tau_mag').value;
    if (tauA!=='') payload.tau_acc = +tauA;
    if (tauM!=='') payload.tau_mag = +tauM;
    payload.motion_bias = document.getElementById('motion_bias').checked ? 1 : 0;
    payload.rest_bias = document.getElementById('rest_bias').checked ? 1 : 0;
    payload.mag_reject = document.getElementById('mag_reject').checked ? 1 : 0;
        payload.use_mag = document.getElementById('use_mag').checked ? 1 : 0;
    sendVqf(payload);
};

document.getElementById('vqfDefaults').onclick = ()=>{
    document.getElementById('tau_acc').value = '3.0';
    document.getElementById('tau_mag').value = '9.0';
    document.getElementById('motion_bias').checked = true;
    document.getElementById('rest_bias').checked = true;
    document.getElementById('mag_reject').checked = true;
        document.getElementById('use_mag').checked = true;
    sendVqf({
        tau_acc: 3.0,
        tau_mag: 9.0,
        motion_bias: 1,
        rest_bias: 1,
            mag_reject: 1,
            use_mag: 1
    });
};

document.getElementById('resetVqf').onclick = ()=>{
    sendCmd({raw:'vqf.reset'});
};

document.getElementById('applyCore').onclick = ()=>{
  const params = {};
  const acc = document.getElementById('acc_diff_th').value;
  const gyr = document.getElementById('gyr_norm_th').value;
  const hys = document.getElementById('hys_frac').value;
  const rest = document.getElementById('min_rest_s').value;
  const motion = document.getElementById('min_motion_s').value;
  if (acc!=='') params.acc_diff_th = +acc;
  if (gyr!=='') params.gyr_norm_th = +gyr;
  if (hys!=='') params.hys_frac = +hys;
  if (rest!=='') params.min_rest_s = +rest;
  if (motion!=='') params.min_motion_s = +motion;
  sendFpaCore(params);
};

document.getElementById('resetCore').onclick = ()=>{
  // Reset to balanced defaults
  document.getElementById('acc_diff_th').value = '0.35';
  document.getElementById('gyr_norm_th').value = '1.0';
  document.getElementById('hys_frac').value = '0.25';
  document.getElementById('min_rest_s').value = '0.08';
  document.getElementById('min_motion_s').value = '0.06';
  sendFpaCore({
    acc_diff_th: 0.35,
    gyr_norm_th: 1.0,
    hys_frac: 0.25,
    min_rest_s: 0.08,
    min_motion_s: 0.06
  });
};

poll();
</script>
</html>
"""

def parse_args():
    p = argparse.ArgumentParser(description="ESP32 FPA Dashboard (TCP + Web UI)")
    p.add_argument("--bind", default="0.0.0.0", help="HTTP bind host (default 0.0.0.0)")
    p.add_argument("--http", type=int, default=8000, help="HTTP port (default 8000)")
    p.add_argument("--tcp-host", default="0.0.0.0", help="ESP32 TCP bind host (default 0.0.0.0)")
    p.add_argument("--tcp-port", type=int, default=12345, help="ESP32 TCP port for board to connect (default 12345)")
    return p.parse_args()


def main():
    args = parse_args()
    state = SharedState()
    bridge = TcpBridge(args.tcp_host, args.tcp_port, state)
    bridge.start()

    app = make_app(bridge, state)
    uvicorn.run(app, host=args.bind, port=args.http, log_level="info")


if __name__ == "__main__":
    main()
