#!/usr/bin/env python3
"""
Debug script to test HR/FC detection on stridelog.csv
Processes the data and outputs detailed diagnostic information.
"""
import sys
import math
from utils.hrfc import HRFCDetector

def load_csv(filepath):
    """Load CSV data: t_ms,ax,ay,az,gx,gy,gz,qw,qx,qy,qz"""
    data = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',')
            if len(parts) != 11:
                print(f"Warning: Skipping malformed line (expected 11 columns, got {len(parts)}): {line[:80]}", file=sys.stderr)
                continue
            try:
                vals = [float(p) for p in parts]
                data.append(vals)
            except ValueError as e:
                print(f"Warning: Skipping non-numeric line: {line[:80]} - {e}", file=sys.stderr)
                continue
    return data

def main():
    # Load data
    filepath = '/home/iquibalh/Documents/cmu/research/HAMMER/hammer-vqf/imu_vqf_stream/stridelog.csv'
    print(f"Loading data from {filepath}...")
    data = load_csv(filepath)
    print(f"Loaded {len(data)} samples")
    
    if not data:
        print("ERROR: No data loaded!")
        return
    
    # Time range
    t_start = data[0][0]
    t_end = data[-1][0]
    duration = (t_end - t_start) / 1000.0
    print(f"Time range: {t_start:.3f} ms to {t_end:.3f} ms ({duration:.3f} seconds)")
    
    # Compute sample rate
    if len(data) > 1:
        dt_vals = [(data[i][0] - data[i-1][0]) for i in range(1, min(100, len(data)))]
        avg_dt = sum(dt_vals) / len(dt_vals)
        sample_rate = 1000.0 / avg_dt if avg_dt > 0 else 0
        print(f"Average dt: {avg_dt:.3f} ms, Sample rate: ~{sample_rate:.1f} Hz")
    
    # Create detector with default parameters
    print("\nInitializing HR/FC detector with parameters:")
    params = {
        'g': 9.81,
        'th_win': 5.0,
        'rc_iters': 100,
        'hyst_frac': 0.23,
        'a_th_min': 2.5,
        'w_th_min': 0.2,
        # 'T0_min': 0.120,
        # 'T1_min': 0.180,
        'T0_min': 0.200,
        'T1_min': 0.250,
        'w_acc': 0.85,
        'w_gyr': 0.80,
        'gyro_units': 'rad'
    }
    for k, v in params.items():
        print(f"  {k}: {v}")
    
    detector = HRFCDetector(**params)
    
    # Process data
    print("\nProcessing samples...")
    all_events = []
    debug_samples = []
    
    for i, row in enumerate(data):
        t_ms, ax, ay, az, gx, gy, gz, qw, qx, qy, qz = row
        
        # Update detector
        detector.update(t_ms, ax, ay, az, gx, gy, gz, qw, qx, qy, qz)
        
        # Get debug info
        dbg = detector.get_debug()
        debug_samples.append(dbg)
        
        # Pop events
        events = detector.pop_events()
        for tag, t_s in events:
            all_events.append((tag, t_s))
            print(f"  EVENT: {tag} at {t_s:.3f} s (sample {i+1}/{len(data)})")
        
        # Print progress every 100 samples
        if (i+1) % 100 == 0:
            print(f"  Processed {i+1}/{len(data)} samples... (t={t_ms/1000:.3f}s, a_err={dbg['a_err']:.3f}, w_norm={dbg['w_norm']:.3f}, r={dbg['r']})")
    
    print(f"\nProcessing complete! Processed {len(data)} samples")
    print(f"\nTotal events detected: {len(all_events)}")
    
    if all_events:
        print("\nEvent Summary:")
        for tag, t_s in all_events:
            print(f"  {tag}: {t_s:.3f} s")
    else:
        print("\n⚠️  NO EVENTS DETECTED!")
        print("\nDiagnostic Analysis:")
        
        # Analyze why no events
        print("\n1. Checking acceleration error (|a|-g):")
        a_errs = [d['a_err'] for d in debug_samples]
        a_ths = [d['a_th'] for d in debug_samples]
        print(f"   Min a_err: {min(a_errs):.3f}")
        print(f"   Max a_err: {max(a_errs):.3f}")
        print(f"   Mean a_err: {sum(a_errs)/len(a_errs):.3f}")
        print(f"   Final a_th: {a_ths[-1]:.3f}")
        
        print("\n2. Checking gyro norm (‖ω‖):")
        w_norms = [d['w_norm'] for d in debug_samples]
        w_ths = [d['w_th'] for d in debug_samples]
        print(f"   Min w_norm: {min(w_norms):.3f}")
        print(f"   Max w_norm: {max(w_norms):.3f}")
        print(f"   Mean w_norm: {sum(w_norms)/len(w_norms):.3f}")
        print(f"   Final w_th: {w_ths[-1]:.3f}")
        
        print("\n3. Checking rest state:")
        r_vals = [d['r'] for d in debug_samples]
        r_sum = sum(r_vals)
        print(f"   Total samples in rest (r=1): {r_sum}/{len(r_vals)}")
        print(f"   Total samples in motion (r=0): {len(r_vals)-r_sum}/{len(r_vals)}")
        
        # Check for state transitions
        transitions = []
        for i in range(1, len(r_vals)):
            if r_vals[i] != r_vals[i-1]:
                transitions.append((i, r_vals[i-1], r_vals[i], debug_samples[i]['t']))
        
        print(f"\n4. State transitions: {len(transitions)}")
        if transitions:
            print("   First 10 transitions:")
            for idx, (i, prev, curr, t) in enumerate(transitions[:10]):
                trans_type = "Motion→Rest (potential FC)" if curr == 1 else "Rest→Motion (potential HR)"
                print(f"     {idx+1}. Sample {i}: {prev}→{curr} at t={t:.3f}s ({trans_type})")
        else:
            print("   ⚠️  No state transitions detected! Check if thresholds are too high/low.")
            
        # Sample some debug data points
        print("\n5. Sample debug data (every 100 samples):")
        print("   t(s)    a_err   a_th    w_norm  w_th    r")
        for i in range(0, len(debug_samples), 100):
            d = debug_samples[i]
            print(f"   {d['t']:6.3f}  {d['a_err']:6.3f}  {d['a_th']:6.3f}  {d['w_norm']:6.3f}  {d['w_th']:6.3f}  {d['r']}")
        
        # Check actual acceleration and gyro from raw data
        print("\n6. Raw sensor data statistics:")
        ax_vals = [row[1] for row in data]
        ay_vals = [row[2] for row in data]
        az_vals = [row[3] for row in data]
        gx_vals = [row[4] for row in data]
        gy_vals = [row[5] for row in data]
        gz_vals = [row[6] for row in data]
        
        a_norms = [math.sqrt(ax*ax + ay*ay + az*az) for ax, ay, az in zip(ax_vals, ay_vals, az_vals)]
        print(f"   ‖a‖: min={min(a_norms):.3f}, max={max(a_norms):.3f}, mean={sum(a_norms)/len(a_norms):.3f}")
        print(f"   Expected g={params['g']:.3f}")
        print(f"   ax: min={min(ax_vals):.3f}, max={max(ax_vals):.3f}")
        print(f"   ay: min={min(ay_vals):.3f}, max={max(ay_vals):.3f}")
        print(f"   az: min={min(az_vals):.3f}, max={max(az_vals):.3f}")
        print(f"   gx: min={min(gx_vals):.3f}, max={max(gx_vals):.3f}")
        print(f"   gy: min={min(gy_vals):.3f}, max={max(gy_vals):.3f}")
        print(f"   gz: min={min(gz_vals):.3f}, max={max(gz_vals):.3f}")

if __name__ == "__main__":
    main()
