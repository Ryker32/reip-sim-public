#!/usr/bin/env python3
"""Trust-ledger figure: one follower's suspicion and trust through a bad-leader fault.

Reproducible from the raw campaign logs, like every other published number.

    python test/_generate_trust_ledger_figure.py            # writes the PDF
    python test/_generate_trust_ledger_figure.py --show     # and reports the trace

TRIAL SELECTION (stated so it can be checked, not chosen for appearance):
  Among the 97 detected REIP bad-leader trials of the 2026-02-28 campaign, take
  the trial whose time_to_detection is closest to the campaign median, breaking
  ties by lowest trial number.  The median is 1.2077 s; trial 90 sits at 1.208 s.
  It is also at the median on first suspicion (0.21 s) and on final coverage
  (100%), and it falls in the modal bin of the leadership-change mechanism scan.
  The rule is computed from results_final_*.json alone; no trace was inspected
  before selecting.

FOLLOWER SELECTION:
  The follower whose trust decays first -- the one whose evidence drives the
  bound.  In trial 90 that is robot 3, which flags on Tier-1 evidence
  (personal_visited, w = 1.0).

WHY COMMAND INDEX AND NOT TIME:
  Theorem 1 is stated in commands, so the bounds overlay directly.  It also
  avoids the 5 Hz trace sampling, which can miss a trust step that occurs
  between samples: in this very trial the JSONL last sampled robot 3 at 0.60
  while the event log shows it reached 0.40 before leadership changed.  The
  stdout log records every accumulator update as an event, so it is exact.
"""
import argparse
import glob
import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CAMPAIGN = os.path.join(
    ROOT, 'experiments', 'run_20260228_014625_multiroom_100trials_all')

sys.path.insert(0, os.path.join(ROOT, 'robot'))
import contextlib, io  # noqa: E402
with contextlib.redirect_stdout(io.StringIO()):
    import reip_node as rn  # noqa: E402

BAD = re.compile(r'\[TRUST\] Bad command #(\d+) \(([^)]*)\): suspicion \+([0-9.]+) = ([0-9.]+)')
DECAY = re.compile(r'\[TRUST\] Trust decayed: trust=([0-9.]+), remaining suspicion=([0-9.]+)')
CHANGE = re.compile(r'\[ELECTION\] (?:Leader \d+ impeached|New leader: (\d+))')


def select_trial():
    """Median time_to_detection among detected bad-leader trials; ties by trial number."""
    results = json.load(open(glob.glob(f'{CAMPAIGN}/results_final_*.json')[0]))
    detected = [r for r in results
                if r['controller'] == 'reip' and r['fault_type'] == 'bad_leader'
                and r['time_to_detection'] is not None]
    lat = sorted(r['time_to_detection'] for r in detected)
    n = len(lat)
    median = lat[n // 2] if n % 2 else (lat[n // 2 - 1] + lat[n // 2]) / 2
    return min(detected, key=lambda r: (abs(r['time_to_detection'] - median), r['trial'])), median, n


def parse_follower(trial_name, fault_robot=1):
    """Exact per-command ledger for the first-decaying follower, from its event log."""
    best = None
    for rid in range(1, 6):
        if rid == fault_robot:
            continue
        path = f'{CAMPAIGN}/logs/{trial_name}/robot_{rid}.log'
        if not os.path.exists(path):
            continue
        lines = open(path, errors='replace').read().split('\n')
        start = next((i for i, l in enumerate(lines) if BAD.search(l)
                      and BAD.search(l).group(1) == '1'), None)
        if start is None:
            continue
        cmds, trust, change_after, tier = [], 1.0, None, None
        series = []
        for l in lines[start:]:
            m = BAD.search(l)
            if m:
                idx, why, _, total = int(m.group(1)), m.group(2), m.group(3), float(m.group(4))
                if idx <= len(cmds):      # second fault reuses the counter; stop here
                    break
                cmds.append(idx)
                tier = tier or why
                series.append({'n': idx, 'S': total, 'T': trust, 'crossed': False})
                continue
            m = DECAY.search(l)
            if m and series:
                trust = float(m.group(1))
                series[-1]['T'] = trust
                series[-1]['crossed'] = True
                continue
            if CHANGE.search(l) and series:
                change_after = series[-1]['n']
                break
        if series and (best is None or series[0]['n'] < 1e9 and
                       len([s for s in series if s['crossed']]) and
                       (best is None or len(series) < len(best[1]))):
            best = (rid, series, change_after, tier)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--show', action='store_true')
    ap.add_argument('--out', default=os.path.join(ROOT, 'paper_figures_urtc', 'trust_ledger.pdf'))
    args = ap.parse_args()

    trial, median, n = select_trial()
    picked = parse_follower(trial['name'], trial.get('fault_robot', 1))
    if not picked:
        sys.exit('could not parse a follower ledger')
    rid, series, change_after, tier = picked

    print(f'selected {trial["name"]} (trial {trial["trial"]}) '
          f'impeach {trial["time_to_detection"]:.3f}s vs campaign median {median:.3f}s of n={n}')
    print(f'follower R{rid}, evidence tier: {tier}')
    for s in series:
        print(f"  cmd {s['n']}: S={s['S']:.2f} T={s['T']:.2f}"
              f"{'  <- crossing' if s['crossed'] else ''}")
    print(f'  leadership change after command {change_after}')
    print(f'  Theorem 1 (Tier-1): n_detect={rn.WORST_CASE_DETECT_T1}, '
          f'n_repl={rn.WORST_CASE_REPLACE_T1}, n_imp={rn.WORST_CASE_IMPEACH_T1}')
    if args.show:
        return

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    xs = [s['n'] for s in series]
    S = [s['S'] for s in series]
    T = [s['T'] for s in series]
    cross = [s['n'] for s in series if s['crossed']]

    plt.rcParams.update({'font.size': 7, 'axes.labelsize': 7, 'legend.fontsize': 6,
                         'xtick.labelsize': 6.5, 'ytick.labelsize': 6.5,
                         'axes.linewidth': 0.6, 'lines.linewidth': 1.1})
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(3.5, 2.35), sharex=True,
                                   gridspec_kw={'height_ratios': [1, 1], 'hspace': 0.18})

    ax1.step(xs, S, where='mid', color='#1f4e79', marker='o', markersize=2.6)
    ax1.axhline(rn.SUSPICION_THRESHOLD, color='#b03030', ls='--', lw=0.8)
    ax1.text(xs[-1] + 0.05, rn.SUSPICION_THRESHOLD, r'$\sigma$', color='#b03030',
             va='center', fontsize=7)
    ax1.set_ylabel(r'suspicion $S_t$')
    ax1.set_ylim(0, max(S) * 1.25)

    ax2.step(xs, T, where='mid', color='#1f4e79', marker='o', markersize=2.6)
    ax2.axhline(rn.TRUST_THRESHOLD, color='#c8791a', ls='--', lw=0.8)
    ax2.axhline(rn.IMPEACHMENT_THRESHOLD, color='#b03030', ls=':', lw=0.8)
    ax2.text(xs[-1] + 0.05, rn.TRUST_THRESHOLD, r'$\tau_{\rm elig}$', color='#c8791a',
             va='center', fontsize=7)
    ax2.text(xs[-1] + 0.05, rn.IMPEACHMENT_THRESHOLD, r'$\tau_{\rm imp}$', color='#b03030',
             va='center', fontsize=7)
    ax2.set_ylabel(r'trust $T_t$')
    ax2.set_xlabel('leader commands since fault injection')
    ax2.set_ylim(0, 1.08)

    for ax in (ax1, ax2):
        for c in cross:
            ax.axvline(c, color='#999999', lw=0.5, alpha=0.7, zorder=0)
        ax.axvline(rn.WORST_CASE_REPLACE_T1, color='#2e7d32', lw=0.9, ls='-.')
        ax.set_xlim(0.5, max(xs) + 0.9)
        ax.grid(alpha=0.18, lw=0.4)
    ax1.text(rn.WORST_CASE_REPLACE_T1 + 0.08, max(S) * 1.10,
             r'$n_{\rm repl}\!=\!5$', color='#2e7d32', fontsize=6.2)
    if change_after:
        ax2.annotate('leader replaced', xy=(change_after, T[-1]),
                     xytext=(change_after - 2.4, 0.30), fontsize=6.2, color='#2e7d32',
                     arrowprops=dict(arrowstyle='->', color='#2e7d32', lw=0.7))

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    fig.savefig(args.out, bbox_inches='tight', pad_inches=0.02)
    print(f'wrote {args.out}')


if __name__ == '__main__':
    main()
