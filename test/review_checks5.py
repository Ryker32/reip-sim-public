"""Discriminate deadlock-rescue mechanism: timing (impeachment before deadlock
forms) vs drift (stale-command displacement). Compares per-seed progress
checkpoints for the 11 clean-catastrophic REIP seeds under clean vs freeze."""
import glob
import json

BASE = r"c:\Users\ryker\research\reip-sim-public\experiments"


def load(run):
    f = glob.glob(rf"{BASE}\{run}\results_final_*.json")[0]
    return json.load(open(f))


rows = load("run_20260228_014625_multiroom_100trials_all") + load(
    "run_20260228_135632_multiroom_100trials_FreezeLeader")

clean = {r["seed"]: r for r in rows if r["controller"] == "reip" and r["fault_type"] is None}
frz = {r["seed"]: r for r in rows if r["controller"] == "reip" and r["fault_type"] == "freeze_leader"}
cat = sorted(s for s in clean if clean[s]["final_coverage"] < 70)


def f(x, nd=1):
    return "  --  " if x is None else f"{x:6.{nd}f}"


print("seed     | CLEAN: cov  t50    t60    t80   FP | FREEZE: cov  t50    t60    t80   impeach_t")
for s in cat:
    c, z = clean[s], frz[s]
    imp = None if z["time_to_detection"] is None else 10.0 + z["time_to_detection"]
    print(f"{s:8d} | {c['final_coverage']:5.1f} {f(c['time_to_50'])} {f(c['time_to_60'])} "
          f"{f(c['time_to_80'])} {c['false_positives']:3d} | "
          f"{z['final_coverage']:6.1f} {f(z['time_to_50'])} {f(z['time_to_60'])} "
          f"{f(z['time_to_80'])} {f(imp, 2)}")

# Healthy-trial reference: when does a normal trial hit these checkpoints?
ok = [clean[s] for s in clean if clean[s]["final_coverage"] >= 99]
from statistics import median
print(f"\nhealthy clean reference: median t50={median(x['time_to_50'] for x in ok):.1f}s, "
      f"t60={median(x['time_to_60'] for x in ok):.1f}s, t80={median(x['time_to_80'] for x in ok):.1f}s")
