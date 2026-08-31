#!/usr/bin/env python3
"""
Aggregate all hardware trial results and compute paper metrics.

Usage:
    python aggregate_hardware_results.py

Outputs:
    - Summary table matching paper format
    - Individual trial details
    - Detection times for fault conditions
"""
import json
import os
import glob
import re
import sys
from collections import defaultdict
from statistics import mean, median, stdev

# Paper arena: 2000x1500mm, multiroom layout
from arena_coverage import (TOTAL_REACHABLE_CELLS, coverage_pct,
                             covered_cells)

# Denominator is derived from DEFAULT_ARENA.is_wall_cell() (see arena_coverage),
# never a literal, so it cannot drift from the geometry the robots enforce.
TOTAL_EXPLORABLE_CELLS = TOTAL_REACHABLE_CELLS
TRIAL_DURATION = 120  # seconds

# Per-robot state logs appear under three naming conventions across the
# hardware sessions.  Earlier sessions wrote "r<N>_robot_<N>_<ts>.jsonl";
# the 2026-03-15 session wrote "r<N>_<controller>_<N>_<ts>.jsonl"; the
# overhead-camera recordings wrote a single combined "robot_states.jsonl".
# Only the first two carry per-robot known_visited_count, which is what the
# coverage metric needs.
LOG_GLOBS = ('r*_robot_*.jsonl', 'r*_*.jsonl')


def warn(trial_dir, reason):
    """Report a skipped trial loudly.  Silent drops previously hid every
    Raft trial from the summary table."""
    print(f"  [SKIP] {trial_dir}: {reason}", file=sys.stderr)


def find_log_files(trial_dir):
    """Return per-robot log files, trying each naming convention in turn."""
    for pattern in LOG_GLOBS:
        files = sorted(glob.glob(os.path.join(trial_dir, pattern)))
        files = [f for f in files if os.path.basename(f) != 'robot_states.jsonl']
        if files:
            return files
    return []

def parse_trial_dir(trial_dir):
    """Parse a single trial directory and extract metrics."""
    meta_path = os.path.join(trial_dir, 'trial_meta.json')
    if not os.path.exists(meta_path):
        warn(trial_dir, "no trial_meta.json (unlabelled recording; controller "
                        "and fault are unknown, cannot be assigned to a table cell)")
        return None
    
    with open(meta_path) as f:
        meta = json.load(f)
    
    controller = meta.get('controller', 'unknown')
    fault_type = meta.get('fault_type', 'none')
    fault1_time = meta.get('fault1_actual_time')
    fault1_robot = meta.get('fault1_robot')
    
    # Parse robot logs
    log_files = find_log_files(trial_dir)
    if not log_files:
        warn(trial_dir, f"trial_meta.json present (controller={controller}, "
                        f"fault={fault_type}) but no per-robot .jsonl matched "
                        f"any of {LOG_GLOBS}")
        return None
    
    all_entries = {}
    for logfile in log_files:
        rid = int(os.path.basename(logfile).split('_')[0][1:])
        entries = []
        with open(logfile) as f:
            for line_num, line in enumerate(f, 1):
                line = line.strip()
                if line:
                    try:
                        entries.append(json.loads(line))
                    except json.JSONDecodeError:
                        # Skip malformed lines
                        continue
        if entries:
            all_entries[rid] = entries
    
    if not all_entries:
        warn(trial_dir, "per-robot logs present but every one was empty or unparseable")
        return None
    
    # Find trial start time (first entry across all robots)
    t0 = min(e[0]['t'] for e in all_entries.values() if e)
    
    # Coverage: max known_visited_count across all robots at end
    # Coverage is the union of reachable cells occupied by at least one robot,
    # reconstructed from the logged trajectories.  This is the definition the
    # paper states, and unlike each node's known_visited_count it does not
    # depend on which peer gossip arrived, so controllers are measured alike.
    covered = covered_cells(all_entries)
    coverage = coverage_pct(all_entries)
    final_known = max(e[-1].get('known_visited_count', 0) for e in all_entries.values() if e)
    if len(covered) > TOTAL_EXPLORABLE_CELLS or coverage > 100.0:
        warn(trial_dir, f"IMPOSSIBLE COVERAGE: {len(covered)} cells covered but only "
                        f"{TOTAL_EXPLORABLE_CELLS} are reachable ({coverage:.1f}%). "
                        f"Excluded from all means.")
        return None
    if final_known > TOTAL_EXPLORABLE_CELLS:
        warn(trial_dir, f"node reported {final_known} known-visited cells, above the "
                        f"{TOTAL_EXPLORABLE_CELLS}-cell reachable set (pre-2026-03-05 "
                        f"five-cell stamping). Trajectory measure used instead.")
    
    # Speed: compute from position changes
    speeds = []
    for rid, entries in all_entries.items():
        if len(entries) < 2:
            continue
        positions = [(e['t'], e['x'], e['y']) for e in entries if 'x' in e and 'y' in e]
        if len(positions) < 2:
            continue
        
        total_dist = 0
        for i in range(1, len(positions)):
            t1, x1, y1 = positions[i-1]
            t2, x2, y2 = positions[i]
            dt = t2 - t1
            if dt > 0:
                dx = x2 - x1
                dy = y2 - y1
                dist = (dx*dx + dy*dy) ** 0.5
                total_dist += dist
        
        if positions:
            duration = positions[-1][0] - positions[0][0]
            if duration > 0:
                avg_speed = total_dist / duration  # mm/s
                speeds.append(avg_speed)
    
    avg_speed = mean(speeds) if speeds else 0
    
    # Detection time: find first trust drop below 0.85 after fault injection
    detection_time = None
    if fault1_time and fault1_robot and controller == 'reip':
        # fault1_time is already absolute, need to convert to relative
        # Find when fault actually happened relative to trial start
        fault_t_rel = None
        for rid, entries in all_entries.items():
            for e in entries:
                # Check if this entry is close to fault injection time
                if abs(e['t'] - fault1_time) < 5.0:  # Within 5s
                    fault_t_rel = e['t'] - t0
                    break
            if fault_t_rel is not None:
                break
        
        if fault_t_rel is None:
            # Fallback: use meta fault time if available
            fault_t_rel = fault1_time - t0 if fault1_time > t0 else 20.0  # Default to 20s
        
        # Now find first trust drop after fault
        for rid, entries in all_entries.items():
            if rid == fault1_robot:
                continue  # Skip the leader itself
            for e in entries:
                t_rel = e['t'] - t0
                if t_rel >= fault_t_rel:
                    trust = e.get('trust_in_leader', 1.0)
                    if trust < 0.85:
                        detection_time = t_rel - fault_t_rel
                        break
            if detection_time is not None:
                break
    
    return {
        'controller': controller,
        'fault': fault_type,
        'trial_dir': trial_dir,
        'coverage': coverage,
        'speed': avg_speed,
        'detection_time': detection_time,
        'fault1_time': fault1_time - t0 if fault1_time else None,
        'n_robots': len(all_entries),
    }


def _dir_date(trial_dir):
    m = re.search(r'_(\d{8})_\d{6}[/\\]?$', trial_dir)
    return m.group(1) if m else None


def _dir_trialnum(trial_dir):
    m = re.search(r'_t(\d+)_\d{8}_\d{6}[/\\]?$', trial_dir)
    return int(m.group(1)) if m else None


RUN_GUIDE = 'HARDWARE_RUN_GUIDE.md'


def load_run_guide():
    """Trial directories named in the contemporaneous hardware run log.

    HARDWARE_RUN_GUIDE.md records, for each numbered trial, the exact
    directory that run produced.  It was written during the session, so it
    is an independent record of which run counted as trial N -- not an
    after-the-fact reconstruction from the results.  Where a trial number
    was attempted more than once, this log is the authority on which
    attempt is the real one.
    """
    if not os.path.exists(RUN_GUIDE):
        return set()
    with open(RUN_GUIDE, encoding='utf8', errors='replace') as fh:
        text = fh.read()
    return set(re.findall(r'trials[\\/]([a-z_]+_t\d+_\d{8}_\d{6})', text))


def select_paper_trials(results):
    """Pick the trial set behind the paper's hardware table.

    Preferred source: the directories named in HARDWARE_RUN_GUIDE.md.
    For conditions the run log does not cover, fall back to the heuristic
    below and say so, since that fallback is inferred rather than recorded.

    Fallback rule, applied per (controller, fault):

      1. Group the condition's trials by session date.
      2. Prefer the most recent session that contains at least 5 distinct
         trial numbers (t1..t5).  Within it, a repeated trial number means
         the run was retried, so the latest timestamp for each trial number
         supersedes earlier attempts.  Take trial numbers 1-5.
      3. If no session has 5 distinct trial numbers (some early sessions
         logged every run as "t1"), fall back to every valid trial from the
         most recent session for that condition.

    Trials rejected by parse_trial_dir (unlabelled, unparseable, or
    impossible coverage) never reach this function.
    """
    guide = load_run_guide()
    by_cond = defaultdict(lambda: defaultdict(list))
    for r in results:
        by_cond[(r['controller'], r['fault'])][_dir_date(r['trial_dir'])].append(r)

    selected = defaultdict(list)
    for cond, by_date in by_cond.items():
        listed = [r for d in by_date for r in by_date[d]
                  if os.path.basename(r['trial_dir'].rstrip('/\\')) in guide]
        if len(listed) >= 5:
            listed.sort(key=lambda r: (_dir_trialnum(r['trial_dir']) or 0))
            selected[cond] = listed[:5]
            print(f"  {cond[0]:6s} {cond[1]:20s} -> {RUN_GUIDE}, {len(listed[:5])} named trials",
                  file=sys.stderr)
            continue
        dated = sorted(d for d in by_date if d)
        if not dated:
            continue
        chosen_date = None
        for d in reversed(dated):
            if len({_dir_trialnum(r['trial_dir']) for r in by_date[d]
                    if _dir_trialnum(r['trial_dir'])}) >= 5:
                chosen_date = d
                break
        if chosen_date:
            latest_per_num = {}
            for r in by_date[chosen_date]:
                n = _dir_trialnum(r['trial_dir'])
                if n is None:
                    continue
                if n not in latest_per_num or r['trial_dir'] > latest_per_num[n]['trial_dir']:
                    latest_per_num[n] = r
            picked = [latest_per_num[n] for n in sorted(latest_per_num) if n <= 5]
            note = f"session {chosen_date}, trials t1-t5 (latest rerun of each)"
        else:
            chosen_date = dated[-1]
            picked = sorted(by_date[chosen_date], key=lambda r: r['trial_dir'])
            note = (f"session {chosen_date}, all {len(picked)} valid runs "
                    f"(condition has no t1-t5 numbering; NOT in {RUN_GUIDE}, "
                    f"so this selection is inferred, not recorded)")
        selected[cond] = picked
        print(f"  {cond[0]:6s} {cond[1]:20s} -> {note}", file=sys.stderr)
    return selected


def main():
    use_all = '--all' in sys.argv
    trial_dirs = sorted(glob.glob('trials/*/'))
    if not trial_dirs:
        print("No trial directories found in 'trials/'")
        return

    print(f"Scanning {len(trial_dirs)} trial directories"
          f"{' (--all: no paper selection)' if use_all else ''}\n")

    all_results = []
    for trial_dir in trial_dirs:
        result = parse_trial_dir(trial_dir)
        if result:
            all_results.append(result)
    print(f"\n  {len(all_results)} of {len(trial_dirs)} trials parsed successfully.",
          file=sys.stderr)

    groups = defaultdict(list)
    if use_all:
        for r in all_results:
            groups[(r['controller'], r['fault'])].append(r)
    else:
        print("\n  Paper trial selection:", file=sys.stderr)
        for cond, picked in select_paper_trials(all_results).items():
            groups[cond] = picked
    print(file=sys.stderr)

    # Print summary table (matching paper format)
    print("=" * 80)
    print("HARDWARE RESULTS SUMMARY (Paper Table Format)")
    print("=" * 80)
    print(f"{'Ctrl.':<8} {'Fault':<15} {'N':>3}  {'Coverage':>10}  {'Speed':>8}  {'Detect':>8}")
    print(f"{'':8} {'':15} {'':3}  {'@120s':>10}  {'(mm/s)':>8}  {'(s)':>8}")
    print("-" * 80)
    
    # Order: REIP first, then Raft; within each: none, bad_leader, freeze_leader, self_injure_leader
    order = [
        ('reip', 'none'),
        ('reip', 'bad_leader'),
        ('reip', 'freeze_leader'),
        ('reip', 'self_injure_leader'),
        ('raft', 'none'),
        ('raft', 'bad_leader'),
        ('raft', 'freeze_leader'),
        ('raft', 'self_injure_leader'),
    ]
    
    summary_data = {}
    for ctrl, fault in order:
        key = (ctrl, fault)
        if key not in groups:
            continue
        
        trials = groups[key]
        n = len(trials)
        coverages = [t['coverage'] for t in trials]
        speeds = [t['speed'] for t in trials]
        detections = [t['detection_time'] for t in trials if t['detection_time'] is not None]
        
        avg_coverage = mean(coverages)
        avg_speed = mean(speeds)
        avg_detect = mean(detections) if detections else None
        
        # Format fault name for paper
        fault_display = {
            'none': 'None',
            'bad_leader': 'Bad Leader',
            'freeze_leader': 'Freeze Ldr',
            'self_injure_leader': 'Self-Injure',
        }.get(fault, fault)
        
        ctrl_display = ctrl.upper()
        detect_str = f"{avg_detect:.1f}" if avg_detect is not None else "---"
        
        print(f"{ctrl_display:<8} {fault_display:<15} {n:>3}  {avg_coverage:>9.1f}%  {avg_speed:>7.0f}  {detect_str:>8}")
        
        summary_data[key] = {
            'n': n,
            'coverage': avg_coverage,
            'speed': avg_speed,
            'detection': avg_detect,
            'trials': trials,
        }
    
    print("=" * 80)
    print()
    
    # Print detailed breakdown
    print("DETAILED BREAKDOWN BY CONDITION")
    print("=" * 80)
    for ctrl, fault in order:
        key = (ctrl, fault)
        if key not in summary_data:
            continue
        
        data = summary_data[key]
        print(f"\n{ctrl.upper()} + {fault}:")
        print(f"  N = {data['n']} trials")
        print(f"  Coverage: {data['coverage']:.1f}% (mean)")
        print(f"  Speed: {data['speed']:.0f} mm/s (mean)")
        if data['detection']:
            print(f"  Detection: {data['detection']:.1f}s (mean)")
        
        # Individual trial values
        print("  Individual trials:")
        for i, trial in enumerate(data['trials'], 1):
            detect_str = f", detect={trial['detection_time']:.1f}s" if trial['detection_time'] else ""
            print(f"    Trial {i}: {trial['coverage']:.1f}%, {trial['speed']:.0f} mm/s{detect_str}")
    
    # LaTeX table format
    print("\n" + "=" * 80)
    print("LATEX TABLE FORMAT (copy into paper)")
    print("=" * 80)
    print("\\begin{tabular}{@{}llcccc@{}}")
    print("\\toprule")
    print("\\textbf{Ctrl.} & \\textbf{Fault} & $N$ & \\textbf{Cov.} &")
    print("\\textbf{Speed} & \\textbf{Detect} \\\\")
    print(" & & & \\textbf{@120s} & \\textbf{(mm/s)} & \\textbf{(s)} \\\\")
    print("\\midrule")
    
    for ctrl, fault in order:
        key = (ctrl, fault)
        if key not in summary_data:
            continue
        
        data = summary_data[key]
        ctrl_display = ctrl.capitalize()
        fault_display = {
            'none': 'None',
            'bad_leader': 'Bad Leader',
            'freeze_leader': 'Freeze Ldr',
            'self_injure_leader': 'Self-Injure',
        }.get(fault, fault)
        
        detect_str = f"{data['detection']:.1f}" if data['detection'] else "---"
        
        print(f"{ctrl_display:<8} & {fault_display:<15} & {data['n']:>3}  & {data['coverage']:>9.1f}\\% & "
              f"{data['speed']:>7.0f} & {detect_str:>8} \\\\")
    
    print("\\bottomrule")
    print("\\end{tabular}")


if __name__ == '__main__':
    main()
