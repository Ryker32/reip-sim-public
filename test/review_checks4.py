"""Q&A prep: why don't clean false impeachments rescue the 11 deadlocked clean seeds?"""
import glob
import json

BASE = r"c:\Users\ryker\research\reip-sim-public\experiments"
f = glob.glob(rf"{BASE}\run_20260228_014625_multiroom_100trials_all\results_final_*.json")[0]
rows = json.load(open(f))

clean = {r["seed"]: r for r in rows if r["controller"] == "reip" and r["fault_type"] is None}
cat = sorted(s for s in clean if clean[s]["final_coverage"] < 70)
ok = [s for s in clean if clean[s]["final_coverage"] >= 70]

print("clean-catastrophic seeds: FP counts and leader changes")
for s in cat:
    r = clean[s]
    print(f"  seed {s}: cov {r['final_coverage']:.1f}, FP {r['false_positives']}, "
          f"leader changes {r['num_leader_changes']}")
fp_cat = sum(clean[s]["false_positives"] for s in cat) / len(cat)
fp_ok = sum(clean[s]["false_positives"] for s in ok) / len(ok)
print(f"\nmean FP: catastrophic trials {fp_cat:.2f}, non-catastrophic {fp_ok:.2f}")
