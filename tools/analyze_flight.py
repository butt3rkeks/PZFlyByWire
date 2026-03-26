"""
HeliFlightLog CSV analyzer — produces a compact summary for Claude to parse.

Usage:
    python analyze_flight.py <path_to_csv> [--full]

Output:
    - Recording metadata (duration, FPS, frame count)
    - Per-second aggregates: position, velocity, sim error, inputs
    - Anomaly detection: oscillation, drift, spikes, gravity comp gaps
    - Segment analysis: acceleration, cruise, braking, hover, turns

The default output fits within 2000 lines. Use --full for raw per-frame dumps
of anomalous segments only (still bounded).
"""

import csv
import sys
import math
import os
from collections import defaultdict

def load_csv(path):
    rows = []
    with open(path, 'r') as f:
        raw = f.read()

    # Handle missing newlines: PZ's write() may not add \n.
    # Detect by checking if there's only 1-2 lines but the content is huge.
    lines = raw.split('\n')
    if len(lines) <= 2 and len(raw) > 1000:
        # No newlines — split on the repeating pattern.
        # The header tells us column names. Data rows start with a digit (frame number).
        # Re-split: find header, then split remaining on frame-number boundaries.
        header_end = raw.find(',ms,')
        if header_end == -1:
            print("ERROR: Cannot find CSV header")
            return []
        # Find the header start
        header_start = raw.rfind('\n', 0, header_end) + 1
        header_line = raw[header_start:].split('\n')[0] if '\n' in raw[header_start:] else raw[header_start:]

        # The header has N columns. Count them.
        header_fields = header_line.strip().split(',')
        if header_fields[0] != 'frame':
            # Header might be embedded in single line — find 'frame,' prefix
            idx = raw.find('frame,')
            header_line = raw[idx:]

        import re
        header_str = 'frame,ms,state,fps,aX,aZ,alt,vX,vY,vZ,sX,sZ,svX,svZ,eX,eZ,dvX,dvZ,dvY,sub,tilt,faOff,yawSim,yawAct,gravComp,dualPath,keys'
        idx = raw.find(header_str)
        if idx == -1:
            print("ERROR: Cannot find expected CSV header")
            return []
        data = raw[idx + len(header_str):]
        # Split between keys value ([UDLRWSad-]+) and next frame number (digits,13-digit-ms,)
        parts = re.split(r'(?<=[UDLRWSad-])(?=\d+,1\d{12},)', data)
        lines = [header_str] + [p for p in parts if p.strip()]
        print(f"  (Recovered {len(lines)-1} rows from newline-less CSV)")

    reader = csv.DictReader(lines)
    for row in reader:
        parsed = {}
        for k, v in row.items():
            if k is None or v is None:
                continue
            k = k.strip()
            try:
                if k in ('state', 'keys'):
                    parsed[k] = v.strip()
                elif k in ('tilt', 'faOff', 'gravComp', 'dualPath', 'sub'):
                    parsed[k] = int(v)
                else:
                    parsed[k] = float(v)
            except (ValueError, TypeError):
                parsed[k] = v.strip() if isinstance(v, str) else v
        if parsed:
            rows.append(parsed)
    return rows


def hz_speed(row):
    return math.sqrt(row.get('vX', 0)**2 + row.get('vZ', 0)**2)


def sim_speed(row):
    return math.sqrt(row.get('svX', 0)**2 + row.get('svZ', 0)**2)


def err_mag(row):
    return math.sqrt(row.get('eX', 0)**2 + row.get('eZ', 0)**2)


def classify_phase(row, prev_row):
    """Classify flight phase from frame data."""
    speed = hz_speed(row)
    tilt = row.get('tilt', 0)
    fa_off = row.get('faOff', 0)
    vy = row.get('vY', 0)
    dvy = row.get('dvY', 0)
    keys = row.get('keys', '-')

    if row.get('state') == 'warmup':
        return 'warmup'
    if fa_off:
        return 'fa-off'
    if 'a' in keys or 'd' in keys:
        if tilt:
            return 'turning'
        return 'yaw-only'
    if tilt and speed < 5:
        return 'accelerating'
    if tilt and speed >= 5:
        return 'cruise'
    if not tilt and speed > 2:
        return 'braking'
    if dvy > 1:
        return 'ascending'
    if dvy < -1:
        return 'descending'
    return 'hover'


def compute_per_second(rows):
    """Aggregate rows into per-second buckets."""
    if not rows:
        return []

    base_ms = rows[0].get('ms', 0)
    buckets = defaultdict(list)
    for row in rows:
        sec = int((row.get('ms', 0) - base_ms) / 1000)
        buckets[sec].append(row)

    results = []
    for sec in sorted(buckets.keys()):
        bucket = buckets[sec]
        n = len(bucket)

        speeds = [hz_speed(r) for r in bucket]
        errs = [err_mag(r) for r in bucket]
        vys = [r.get('vY', 0) for r in bucket]
        alts = [r.get('alt', 0) for r in bucket]
        fpss = [r.get('fps', 60) for r in bucket]
        subs = [r.get('sub', 1) for r in bucket]
        yaw_diffs = [abs(r.get('yawSim', 0) - r.get('yawAct', 0)) for r in bucket]

        # Phase distribution
        phases = defaultdict(int)
        for i, r in enumerate(bucket):
            prev = bucket[i-1] if i > 0 else r
            phases[classify_phase(r, prev)] += 1
        dominant_phase = max(phases, key=phases.get)

        # Key distribution
        key_counts = defaultdict(int)
        for r in bucket:
            for ch in r.get('keys', '-'):
                if ch != '-':
                    key_counts[ch] += 1

        results.append({
            'sec': sec,
            'frames': n,
            'fps_avg': sum(fpss) / n,
            'speed_avg': sum(speeds) / n,
            'speed_max': max(speeds),
            'speed_min': min(speeds),
            'err_avg': sum(errs) / n,
            'err_max': max(errs),
            'vy_avg': sum(vys) / n,
            'vy_min': min(vys),
            'vy_max': max(vys),
            'alt_avg': sum(alts) / n,
            'alt_min': min(alts),
            'alt_max': max(alts),
            'yaw_diff_avg': sum(yaw_diffs) / n,
            'yaw_diff_max': max(yaw_diffs),
            'sub_zero_pct': sum(1 for s in subs if s == 0) / n * 100,
            'phase': dominant_phase,
            'keys': ''.join(sorted(key_counts.keys())) or '-',
        })

    return results


def detect_anomalies(rows):
    """Detect flight anomalies that indicate bugs."""
    anomalies = []

    for i, row in enumerate(rows):
        frame = row.get('frame', i)
        speed = hz_speed(row)
        err = err_mag(row)
        vy = row.get('vY', 0)

        # 1. Position error spike (> 8 meters = close to MAX_POSITION_ERROR saturation)
        if err > 8.0:
            anomalies.append(('ERR_SPIKE', frame, f'err={err:.2f}m'))

        # 2. Vertical velocity spike during hover (gravity comp failure)
        if row.get('gravComp') and abs(row.get('dvY', 0)) < 0.1 and abs(vy) > 2.0:
            anomalies.append(('GRAV_DRIFT', frame, f'vY={vy:.2f} during hover'))

        # 3. Speed spike (> 500 m/s = beyond max horizontal speed)
        if speed > 500:
            anomalies.append(('SPEED_SPIKE', frame, f'speed={speed:.1f}m/s'))

        # 4. Yaw divergence (sim vs actual > 5 degrees during straight flight)
        yaw_diff = abs(row.get('yawSim', 0) - row.get('yawAct', 0))
        if yaw_diff > 180:
            yaw_diff = 360 - yaw_diff
        keys = row.get('keys', '-')
        if yaw_diff > 5.0 and 'a' not in keys and 'd' not in keys:
            anomalies.append(('YAW_DRIFT', frame, f'yawDiff={yaw_diff:.1f}deg'))

        # 5. Sideways velocity during braking (drift bug)
        if not row.get('tilt') and not row.get('faOff') and speed > 5.0:
            # Check if velocity direction diverges from last desired direction
            if i > 0:
                prev = rows[i-1]
                prev_speed = hz_speed(prev)
                if prev_speed > speed + 5:  # decelerating rapidly
                    # Check lateral component relative to heading
                    pass  # Complex check, skip for now

        # 6. Sub-step zero at moderate FPS (< 90 FPS shouldn't have zero sub-steps)
        if row.get('sub', 1) == 0 and row.get('fps', 60) < 90:
            anomalies.append(('SUBSTEP_ZERO', frame, f'fps={row.get("fps", 0):.0f}'))

        # 7. Oscillation detection: velocity sign change in Y within 3 frames
        if i >= 2:
            vy_0 = rows[i-2].get('vY', 0)
            vy_1 = rows[i-1].get('vY', 0)
            vy_2 = vy
            if (vy_0 > 0.5 and vy_1 < -0.5 and vy_2 > 0.5) or \
               (vy_0 < -0.5 and vy_1 > 0.5 and vy_2 < -0.5):
                anomalies.append(('V_OSCILLATION', frame, f'vY={vy_0:.1f}->{vy_1:.1f}->{vy_2:.1f}'))

    return anomalies


def detect_stopping_quality(rows):
    """Analyze braking events: find transitions from moving to stopped."""
    events = []
    i = 0
    while i < len(rows) - 1:
        speed = hz_speed(rows[i])
        tilt = rows[i].get('tilt', 0)

        # Detect braking start: tilt goes from 1 to 0 while speed > 5
        if tilt == 0 and speed > 5.0 and i > 0 and rows[i-1].get('tilt', 0) == 1:
            start_frame = rows[i].get('frame', i)
            start_speed = speed
            peak_err = 0
            # Track until speed < 0.5 or tilt resumes
            j = i
            while j < len(rows) and hz_speed(rows[j]) > 0.5 and not rows[j].get('tilt', 0):
                peak_err = max(peak_err, err_mag(rows[j]))
                j += 1
            if j < len(rows):
                end_frame = rows[j].get('frame', j)
                duration_ms = rows[j].get('ms', 0) - rows[i].get('ms', 0)
                events.append({
                    'start_frame': start_frame,
                    'start_speed': start_speed,
                    'stop_frames': end_frame - start_frame,
                    'stop_ms': duration_ms,
                    'peak_error': peak_err,
                })
            i = j
        else:
            i += 1

    return events


def print_summary(rows, per_sec, anomalies, stop_events, full=False):
    """Print compact analysis summary."""
    if not rows:
        print("ERROR: No data rows found.")
        return

    duration_ms = rows[-1].get('ms', 0) - rows[0].get('ms', 0)
    duration_s = duration_ms / 1000.0
    avg_fps = sum(r.get('fps', 60) for r in rows) / len(rows)
    max_speed = max(hz_speed(r) for r in rows)
    max_alt = max(r.get('alt', 0) for r in rows)
    min_alt = min(r.get('alt', 0) for r in rows)

    print("=" * 70)
    print("HELICOPTER FLIGHT LOG ANALYSIS")
    print("=" * 70)
    print(f"Frames: {len(rows)}  |  Duration: {duration_s:.1f}s  |  Avg FPS: {avg_fps:.0f}")
    print(f"Max speed: {max_speed:.1f} m/s ({max_speed*3.6:.0f} km/h)  |  Alt range: {min_alt:.2f} - {max_alt:.2f}")
    print()

    # Phase distribution
    phase_counts = defaultdict(int)
    for i, row in enumerate(rows):
        prev = rows[i-1] if i > 0 else row
        phase_counts[classify_phase(row, prev)] += 1
    print("PHASE DISTRIBUTION:")
    for phase, count in sorted(phase_counts.items(), key=lambda x: -x[1]):
        pct = count / len(rows) * 100
        print(f"  {phase:15s} {count:6d} frames ({pct:5.1f}%)")
    print()

    # Anomaly summary
    print(f"ANOMALIES: {len(anomalies)} detected")
    if anomalies:
        # Group by type
        by_type = defaultdict(list)
        for atype, frame, desc in anomalies:
            by_type[atype].append((frame, desc))
        for atype, items in sorted(by_type.items()):
            print(f"  {atype}: {len(items)}x", end="")
            if len(items) <= 3:
                for frame, desc in items:
                    print(f"  [f{frame}: {desc}]", end="")
            else:
                for frame, desc in items[:2]:
                    print(f"  [f{frame}: {desc}]", end="")
                print(f"  ... +{len(items)-2} more", end="")
            print()
    print()

    # Stopping events
    print(f"BRAKING EVENTS: {len(stop_events)}")
    for evt in stop_events:
        print(f"  f{evt['start_frame']}: {evt['start_speed']:.1f}m/s -> stop in "
              f"{evt['stop_frames']}f ({evt['stop_ms']}ms), peak_err={evt['peak_error']:.2f}m")
    print()

    # Per-second table
    print("PER-SECOND TIMELINE:")
    print(f"{'sec':>4s} {'fps':>4s} {'spd':>6s} {'sMax':>6s} {'err':>5s} {'eMax':>5s} "
          f"{'vY':>6s} {'alt':>5s} {'yawD':>5s} {'s0%':>4s} {'phase':>12s} {'keys':>6s}")
    print("-" * 80)
    for s in per_sec:
        print(f"{s['sec']:4d} {s['fps_avg']:4.0f} {s['speed_avg']:6.1f} {s['speed_max']:6.1f} "
              f"{s['err_avg']:5.2f} {s['err_max']:5.2f} {s['vy_avg']:6.2f} {s['alt_avg']:5.2f} "
              f"{s['yaw_diff_avg']:5.2f} {s['sub_zero_pct']:4.0f} {s['phase']:>12s} {s['keys']:>6s}")
    print()

    # Gravity compensation check
    hover_rows = [r for r in rows if abs(r.get('dvY', 0)) < 0.1 and r.get('gravComp')]
    if hover_rows:
        hover_vys = [r.get('vY', 0) for r in hover_rows]
        avg_vy = sum(hover_vys) / len(hover_vys)
        max_vy = max(hover_vys)
        min_vy = min(hover_vys)
        print(f"HOVER GRAVITY CHECK ({len(hover_rows)} frames):")
        print(f"  vY: avg={avg_vy:.4f}  min={min_vy:.4f}  max={max_vy:.4f}")
        if abs(avg_vy) > 0.1:
            print(f"  WARNING: Avg vertical velocity during hover is {avg_vy:.4f} (should be ~0)")
        else:
            print(f"  OK: Gravity compensation stable")
        print()

    # Heading stability check
    straight_rows = [r for r in rows if 'a' not in r.get('keys', '') and 'd' not in r.get('keys', '')
                     and hz_speed(r) > 5.0]
    if straight_rows:
        yaw_diffs = [abs(r.get('yawSim', 0) - r.get('yawAct', 0)) for r in straight_rows]
        yaw_diffs = [d if d <= 180 else 360 - d for d in yaw_diffs]
        avg_yaw_diff = sum(yaw_diffs) / len(yaw_diffs)
        max_yaw_diff = max(yaw_diffs)
        print(f"HEADING STABILITY ({len(straight_rows)} straight-flight frames):")
        print(f"  yaw diff: avg={avg_yaw_diff:.3f}deg  max={max_yaw_diff:.3f}deg")
        if max_yaw_diff > 3.0:
            print(f"  WARNING: Heading drift detected (max {max_yaw_diff:.1f}deg)")
        else:
            print(f"  OK: Yaw hard lock stable")
        print()

    # MPC tracking check
    moving_rows = [r for r in rows if hz_speed(r) > 2.0 and r.get('tilt')]
    if moving_rows:
        errs = [err_mag(r) for r in moving_rows]
        avg_err = sum(errs) / len(errs)
        max_err = max(errs)
        print(f"MPC TRACKING ({len(moving_rows)} moving+tilt frames):")
        print(f"  position error: avg={avg_err:.3f}m  max={max_err:.3f}m")
        if avg_err > 3.0:
            print(f"  WARNING: Average error high ({avg_err:.1f}m)")
        else:
            print(f"  OK: Sim model tracks well")
        print()

    # Full anomaly dump
    if full and anomalies:
        print("=" * 70)
        print("ANOMALY DETAIL (raw frames)")
        print("=" * 70)
        seen_frames = set()
        for atype, frame, desc in anomalies[:100]:  # cap at 100
            if frame in seen_frames:
                continue
            seen_frames.add(frame)
            # Find the frame in rows
            for r in rows:
                if r.get('frame') == frame:
                    print(f"f{frame} [{atype}] {desc}")
                    print(f"  pos=({r.get('aX',0):.2f},{r.get('aZ',0):.2f}) alt={r.get('alt',0):.2f} "
                          f"vel=({r.get('vX',0):.2f},{r.get('vY',0):.2f},{r.get('vZ',0):.2f}) "
                          f"sim=({r.get('sX',0):.2f},{r.get('sZ',0):.2f}) "
                          f"err=({r.get('eX',0):.2f},{r.get('eZ',0):.2f}) "
                          f"keys={r.get('keys','-')}")
                    break


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <HeliFlightLog.csv> [--full]")
        sys.exit(1)

    path = sys.argv[1]
    full = '--full' in sys.argv

    if not os.path.exists(path):
        print(f"ERROR: File not found: {path}")
        sys.exit(1)

    print(f"Loading {path}...")
    rows = load_csv(path)
    print(f"Loaded {len(rows)} frames.")

    per_sec = compute_per_second(rows)
    anomalies = detect_anomalies(rows)
    stop_events = detect_stopping_quality(rows)
    print_summary(rows, per_sec, anomalies, stop_events, full=full)


if __name__ == '__main__':
    main()
