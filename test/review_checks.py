"""Checks for reviewer questions: freeze-rescue mechanism, paired Wilcoxon
REIP-vs-Raft, and Raft clean-condition leader churn."""
import glob
import json

import numpy as np
from scipy.stats import wilcoxon

BASE = r"c:\Users\ryker\research\reip-sim-public\experiments"


def load(run):
    f = glob.glob(rf"{BASE}\{run}\results_final_*.json")[0]
    return json.load(open(f))


rows = load("run_20260228_014625_multiroom_100trials_all") + load(
    "run_20260228_135632_multiroom_100trials_FreezeLeader")


def get(ctrl, fault):
    return {r["seed"]: r for r in rows if r["controller"] == ctrl and r["fault_type"] == fault}


print("=== 1. FREEZE-RESCUE CHECK ===")
clean = get("reip", None)
freeze = get("reip", "freeze_leader")
shared = set(clean) & set(freeze)
print(f"shared seeds clean/freeze: {len(shared)}")
cat_clean = {s for s in shared if clean[s]["final_coverage"] < 70}
cat_freeze = {s for s in shared if freeze[s]["final_coverage"] < 70}
print(f"catastrophic clean: {sorted(cat_clean)}")
print(f"catastrophic freeze: {sorted(cat_freeze)}")
rescued = cat_clean - cat_freeze
print(f"rescued by freeze condition: {len(rescued)}")
for s in sorted(rescued):
    r = freeze[s]
    print(f"  seed {s}: freeze cov {r['final_coverage']:.1f} "
          f"(clean was {clean[s]['final_coverage']:.1f}), "
          f"impeached at {r['time_to_detection']}s, FP {r['false_positives']}")

print("\n=== 2. PAIRED WILCOXON REIP vs RAFT ===")
for fault in ("bad_leader", "freeze_leader"):
    a = get("reip", fault)
    b = get("raft", fault)
    seeds = sorted(set(a) & set(b))
    x = [a[s]["final_coverage"] for s in seeds]
    y = [b[s]["final_coverage"] for s in seeds]
    w = wilcoxon(x, y)
    print(f"{fault}: n={len(seeds)}, Wilcoxon p={w.pvalue:.2e} "
          f"(Mann-Whitney in paper: 5.0e-18 / 4.8e-28)")

print("\n=== 3. RAFT CLEAN-CONDITION CHURN ===")
sample = next(r for r in rows if r["controller"] == "raft" and r["fault_type"] is None)
print("available fields:", sorted(sample.keys()))
raft_clean = get("raft", None)
cats = [r for r in raft_clean.values() if r["final_coverage"] < 70]
print(f"raft clean catastrophic: {len(cats)}/{len(raft_clean)}")
for key in ("num_elections", "leader_changes", "elections", "num_leader_changes",
            "false_positives", "time_to_detection"):
    if key in sample:
        vals_cat = [r.get(key) for r in cats]
        vals_all = [r.get(key) for r in raft_clean.values()]
        print(f"  {key}: catastrophic trials {vals_cat[:12]}")
        nums = [v for v in vals_all if isinstance(v, (int, float))]
        if nums:
            print(f"    all-trials mean {np.mean(nums):.2f}")
