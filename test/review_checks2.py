"""Follow-up: Raft clean catastrophic breakdown + overlap with REIP/Dec."""
import glob
import json

BASE = r"c:\Users\ryker\research\reip-sim-public\experiments"


def load(run):
    f = glob.glob(rf"{BASE}\{run}\results_final_*.json")[0]
    return json.load(open(f))


rows = load("run_20260228_014625_multiroom_100trials_all")


def cat_seeds(ctrl):
    return {r["seed"]: r for r in rows
            if r["controller"] == ctrl and r["fault_type"] is None
            and r["final_coverage"] < 70}


raft, reip, dec = cat_seeds("raft"), cat_seeds("reip"), cat_seeds("decentralized")
print(f"clean catastrophic: raft {len(raft)}, reip {len(reip)}, dec {len(dec)}")
print(f"raft & reip overlap: {len(set(raft) & set(reip))}")
print(f"raft & dec overlap: {len(set(raft) & set(dec))}")
churn = [s for s, r in raft.items() if r["num_leader_changes"] > 0]
nochurn = [s for s, r in raft.items() if r["num_leader_changes"] == 0]
print(f"raft catastrophic with leader changes: {len(churn)}, without: {len(nochurn)}")
print("churn trials:", [(s, raft[s]["num_leader_changes"], round(raft[s]["final_coverage"], 1))
                        for s in churn])
