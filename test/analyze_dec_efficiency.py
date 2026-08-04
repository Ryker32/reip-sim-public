"""Paired efficiency comparison: REIP vs Decentralized (clean condition).

Tests whether REIP's coordination advantage over leaderless exploration is
supported by time-to-threshold and checkpoint-coverage metrics, using paired
seeds from the paper's main campaign.

Usage: python test/analyze_dec_efficiency.py <results_final.json>
"""
import json
import sys
from statistics import mean, median

from scipy.stats import wilcoxon


def main(path):
    results = json.load(open(path))
    if isinstance(results, dict):
        results = results["results"]
    rows = {}
    for r in results:
        if r["fault_type"] is None:
            rows[(r["controller"], r["seed"])] = r

    seeds = sorted(s for c, s in rows if c == "reip" and ("decentralized", s) in rows)
    print(f"Paired clean trials: N={len(seeds)}\n")

    for metric in ("time_to_50", "time_to_60", "time_to_80", "coverage_at_90s", "final_coverage"):
        reip = [rows[("reip", s)][metric] for s in seeds]
        dec = [rows[("decentralized", s)][metric] for s in seeds]
        if metric.startswith("time_to"):
            # Censored trials (never reached threshold): count them, then compare
            # pairs where both controllers reached it.
            r_miss = sum(1 for v in reip if v is None)
            d_miss = sum(1 for v in dec if v is None)
            pairs = [(a, b) for a, b in zip(reip, dec) if a is not None and b is not None]
            a, b = zip(*pairs)
            w = wilcoxon(a, b)
            print(f"{metric:16s} REIP med {median(a):6.1f}s vs Dec med {median(b):6.1f}s "
                  f"(paired n={len(pairs)}; never-reached: REIP {r_miss}, Dec {d_miss}; "
                  f"Wilcoxon p={w.pvalue:.3f})")
        else:
            w = wilcoxon(reip, dec) if any(a != b for a, b in zip(reip, dec)) else None
            print(f"{metric:16s} REIP mean {mean(reip):5.1f} / med {median(reip):5.1f} vs "
                  f"Dec mean {mean(dec):5.1f} / med {median(dec):5.1f} "
                  f"(Wilcoxon p={w.pvalue:.3f})" if w else "identical")


if __name__ == "__main__":
    main(sys.argv[1])
