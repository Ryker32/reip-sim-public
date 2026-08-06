"""Check hardware clean-trial speed profile: do robots idle after coverage completes?"""
import glob
import json
import math
import os

trial_dir = r"c:\Users\ryker\research\reip-sim-public\trials\reip_none_t1_20260303_190129"
files = glob.glob(os.path.join(trial_dir, "*.jsonl"))
print(f"{os.path.basename(trial_dir)}: {len(files)} robot logs")

for f in files[:5]:
    pts = []
    for line in open(f):
        try:
            r = json.loads(line)
        except json.JSONDecodeError:
            continue
        t = r.get("t") or r.get("time") or r.get("timestamp")
        x, y = r.get("x"), r.get("y")
        if t is not None and x is not None:
            pts.append((float(t), float(x), float(y)))
    if not pts:
        print(f"  {os.path.basename(f)}: no t/x/y fields; keys={sorted(json.loads(open(f).readline()).keys())}")
        continue
    t0 = pts[0][0]
    halves = {"first60": [], "last30": []}
    for (t1, x1, y1), (t2, x2, y2) in zip(pts, pts[1:]):
        dt = t2 - t1
        if dt <= 0:
            continue
        v = math.hypot(x2 - x1, y2 - y1) / dt
        rel = t1 - t0
        if rel < 60:
            halves["first60"].append(v)
        if rel > 90:
            halves["last30"].append(v)
    f60 = sum(halves["first60"]) / max(1, len(halves["first60"]))
    l30 = sum(halves["last30"]) / max(1, len(halves["last30"]))
    print(f"  {os.path.basename(f)}: mean speed first 60s = {f60:.0f} mm/s, last 30s = {l30:.0f} mm/s")
