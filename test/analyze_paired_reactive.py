"""Paired analysis: proactive REIP vs reactive ablation (same build, same seeds).

Usage:
    python test/analyze_paired_reactive.py <proactive_results.json> <reactive_results.json>

Pairs trials by (fault condition, seed) and reports coverage, detection rate,
detection latency, and false positives, with Wilcoxon signed-rank (paired) and
Mann-Whitney U tests plus rank-biserial effect size on final coverage.
"""
import json
import sys
from statistics import mean, median, stdev

try:
    from scipy.stats import mannwhitneyu, wilcoxon
    HAVE_SCIPY = True
except ImportError:
    HAVE_SCIPY = False


def load(path):
    d = json.load(open(path))
    results = d["results"] if isinstance(d, dict) and "results" in d else d
    out = {}
    for r in results:
        cond = r["fault_type"] or "clean"
        out[(cond, r["seed"])] = r
    return out


def fmt(x, nd=1):
    return "N/A" if x is None else f"{x:.{nd}f}"


def summarize(rows, label):
    cov = [r["final_coverage"] for r in rows]
    det = [r["time_to_detection"] for r in rows if r["time_to_detection"] is not None]
    sus = [r["time_to_first_suspicion"] for r in rows if r["time_to_first_suspicion"] is not None]
    fp = [r["false_positives"] for r in rows]
    print(f"  {label:<10} cov {mean(cov):5.1f}+/-{stdev(cov):4.1f} (med {median(cov):5.1f})"
          f"  det {len(det)}/{len(rows)}"
          f"  det-lat med {fmt(median(det) if det else None)}s"
          f"  sus med {fmt(median(sus) if sus else None, 2)}s"
          f"  FP/trial {mean(fp):.1f}")
    return cov


def main(pro_path, rea_path):
    pro, rea = load(pro_path), load(rea_path)
    conds = sorted({c for c, _ in pro} | {c for c, _ in rea})
    for cond in conds:
        seeds = sorted(s for c, s in pro if c == cond and (cond, s) in rea)
        p_rows = [pro[(cond, s)] for s in seeds]
        r_rows = [rea[(cond, s)] for s in seeds]
        print(f"\n{cond}  (paired N={len(seeds)})")
        p_cov = summarize(p_rows, "proactive")
        r_cov = summarize(r_rows, "reactive")
        diffs = [p - r for p, r in zip(p_cov, r_cov)]
        print(f"  paired coverage delta: mean {mean(diffs):+.1f} pp, median {median(diffs):+.1f} pp")
        if HAVE_SCIPY and any(d != 0 for d in diffs):
            w = wilcoxon(p_cov, r_cov)
            u = mannwhitneyu(p_cov, r_cov, alternative="two-sided")
            n1 = n2 = len(seeds)
            rbc = 1 - 2 * u.statistic / (n1 * n2)  # rank-biserial correlation
            print(f"  Wilcoxon signed-rank p={w.pvalue:.2e}; "
                  f"Mann-Whitney U p={u.pvalue:.2e}, rank-biserial r={rbc:+.3f}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
