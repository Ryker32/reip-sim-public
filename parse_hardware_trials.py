#!/usr/bin/env python3
"""
Parse hardware trials based on the trial folder names from HARDWARE_RUN_GUIDE.md
"""
import json
import os
import glob
from collections import defaultdict
from statistics import mean

# Map from guide - trial folder names to (controller, fault, trial_num)
TRIAL_MAP = {
    # REIP clean
    'reip_none_t1_20260315': ('reip', 'none', 1),
    'reip_none_t2_20260315': ('reip', 'none', 2),
    'reip_none_t3_20260315': ('reip', 'none', 3),
    'reip_none_t4_20260315': ('reip', 'none', 4),
    'reip_none_t5_20260315': ('reip', 'none', 5),
    
    # REIP bad_leader
    'reip_bad_leader_t1_20260315_185118': ('reip', 'bad_leader', 1),
    'reip_bad_leader_t2_20260315_185439': ('reip', 'bad_leader', 2),
    'reip_bad_leader_t3_20260315_185801': ('reip', 'bad_leader', 3),
    'reip_bad_leader_t4_20260315_190114': ('reip', 'bad_leader', 4),
    'reip_bad_leader_t5_20260315_191258': ('reip', 'bad_leader', 5),
    
    # REIP freeze_leader
    'reip_freeze_leader_t1_20260315_191636': ('reip', 'freeze_leader', 1),
    'reip_freeze_leader_t2_20260315_192025': ('reip', 'freeze_leader', 2),
    'reip_freeze_leader_t3_20260315_192847': ('reip', 'freeze_leader', 3),
    'reip_freeze_leader_t4_20260315_193148': ('reip', 'freeze_leader', 4),
    'reip_freeze_leader_t5_20260315_193633': ('reip', 'freeze_leader', 5),
    
    # REIP self_injure_leader
    'reip_self_injure_leader_t1_20260315_194004': ('reip', 'self_injure_leader', 1),
    'reip_self_injure_leader_t2_20260315_194349': ('reip', 'self_injure_leader', 2),
    'reip_self_injure_leader_t3_20260315_194659': ('reip', 'self_injure_leader', 3),
    'reip_self_injure_leader_t4_20260315_195024': ('reip', 'self_injure_leader', 4),
    'reip_self_injure_leader_t5_20260315_195345': ('reip', 'self_injure_leader', 5),
    
    # Raft none
    'raft_none_t1_20260315_182753': ('raft', 'none', 1),
    'raft_none_t2_20260315_195714': ('raft', 'none', 2),
    'raft_none_t3_20260315_200055': ('raft', 'none', 3),
    'raft_none_t4_20260315_200445': ('raft', 'none', 4),
    'raft_none_t5_20260315_200835': ('raft', 'none', 5),
    
    # Raft bad_leader
    'raft_bad_leader_t1_20260315_201736': ('raft', 'bad_leader', 1),
    'raft_bad_leader_t2_20260315_202146': ('raft', 'bad_leader', 2),
    'raft_bad_leader_t3_20260315_202706': ('raft', 'bad_leader', 3),
    'raft_bad_leader_t4_20260315_203033': ('raft', 'bad_leader', 4),
    'raft_bad_leader_t5_20260315_203417': ('raft', 'bad_leader', 5),
    
    # Raft freeze_leader
    'raft_freeze_leader_t1_20260315_203754': ('raft', 'freeze_leader', 1),
    'raft_freeze_leader_t2_20260315_204127': ('raft', 'freeze_leader', 2),
    'raft_freeze_leader_t3_20260315_204418': ('raft', 'freeze_leader', 3),
    'raft_freeze_leader_t4_20260315_204744': ('raft', 'freeze_leader', 4),
    'raft_freeze_leader_t5_20260315_205032': ('raft', 'freeze_leader', 5),
    
    # Raft self_injure_leader
    'raft_self_injure_leader_t1_20260315_210036': ('raft', 'self_injure_leader', 1),
    'raft_self_injure_leader_t2_20260315_210344': ('raft', 'self_injure_leader', 2),
    'raft_self_injure_leader_t3_20260315_210650': ('raft', 'self_injure_leader', 3),
    'raft_self_injure_leader_t4_20260315_211209': ('raft', 'self_injure_leader', 4),
    'raft_self_injure_leader_t5_20260315_211523': ('raft', 'self_injure_leader', 5),
}

from arena_coverage import TOTAL_REACHABLE_CELLS

# Derived from DEFAULT_ARENA.is_wall_cell(), never a literal.
TOTAL_EXPLORABLE_CELLS = TOTAL_REACHABLE_CELLS

def find_trial_folder(pattern):
    """Find trial folder matching pattern."""
    # Try exact match first
    exact = f'trials/{pattern}'
    if os.path.exists(exact):
        return exact
    
    # Try with timestamp variations
    dirs = glob.glob(f'trials/{pattern}*')
    if dirs:
        return dirs[0]
    
    # Try timestamp-only folders (20260315_HHMMSS)
    # Extract timestamp from pattern
    if '20260315' in pattern:
        parts = pattern.split('_')
        if len(parts) >= 2:
            timestamp = parts[-1] if parts[-1].isdigit() else None
            if timestamp:
                dirs = glob.glob(f'trials/20260315_{timestamp}*')
                if dirs:
                    return dirs[0]
    
    return None

def parse_trial(trial_dir):
    """Parse a trial directory."""
    # Look for robot log files
    log_files = glob.glob(os.path.join(trial_dir, 'r*_*.jsonl'))
    if not log_files:
        # Try robot_states.jsonl
        alt_log = os.path.join(trial_dir, 'robot_states.jsonl')
        if os.path.exists(alt_log):
            log_files = [alt_log]
        else:
            return None
    
    # Parse logs
    all_entries = {}
    for logfile in log_files:
        # Extract robot ID from filename
        basename = os.path.basename(logfile)
        if basename.startswith('r') and '_' in basename:
            try:
                rid = int(basename[1:].split('_')[0])
            except:
                rid = 1  # Default
        else:
            rid = 1
        
        entries = []
        with open(logfile) as f:
            for line in f:
                line = line.strip()
                if line:
                    try:
                        entries.append(json.loads(line))
                    except:
                        continue
        if entries:
            all_entries[rid] = entries
    
    if not all_entries:
        return None
    
    # Find trial start
    t0 = min(e[0].get('t', e[0].get('timestamp', 0)) for e in all_entries.values() if e)
    
    # Coverage: max known_visited_count
    final_known = 0
    for entries in all_entries.values():
        if entries:
            last = entries[-1]
            known = last.get('known_visited_count', last.get('coverage', 0))
            if isinstance(known, float) and known <= 1.0:
                known = int(known * TOTAL_EXPLORABLE_CELLS)
            final_known = max(final_known, known)
    
    coverage_pct = final_known / TOTAL_EXPLORABLE_CELLS * 100
    
    # Speed: from position changes
    speeds = []
    for rid, entries in all_entries.items():
        positions = [(e.get('t', e.get('timestamp', 0)), e.get('x', 0), e.get('y', 0)) 
                     for e in entries if 'x' in e and 'y' in e]
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
                avg_speed = total_dist / duration
                speeds.append(avg_speed)
    
    avg_speed = mean(speeds) if speeds else 0
    
    # Detection time: find first trust drop (for REIP only)
    detection_time = None
    # This would need fault injection time from meta or logs
    
    return {
        'coverage': coverage_pct,
        'speed': avg_speed,
        'detection_time': detection_time,
    }

def main():
    # Group by (controller, fault)
    groups = defaultdict(list)
    
    for pattern, (ctrl, fault, trial_num) in TRIAL_MAP.items():
        trial_dir = find_trial_folder(pattern)
        if not trial_dir:
            print(f"WARNING: Could not find folder for {pattern}")
            continue
        
        result = parse_trial(trial_dir)
        if result:
            result['controller'] = ctrl
            result['fault'] = fault
            result['trial_num'] = trial_num
            groups[(ctrl, fault)].append(result)
    
    # Print summary
    print("=" * 80)
    print("HARDWARE RESULTS SUMMARY")
    print("=" * 80)
    print(f"{'Ctrl.':<8} {'Fault':<15} {'N':>3}  {'Coverage':>10}  {'Speed':>8}")
    print(f"{'':8} {'':15} {'':3}  {'@120s':>10}  {'(mm/s)':>8}")
    print("-" * 80)
    
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
    
    for ctrl, fault in order:
        key = (ctrl, fault)
        if key not in groups:
            continue
        
        trials = groups[key]
        n = len(trials)
        coverages = [t['coverage'] for t in trials]
        speeds = [t['speed'] for t in trials]
        
        avg_cov = mean(coverages)
        avg_speed = mean(speeds)
        
        fault_display = {
            'none': 'None',
            'bad_leader': 'Bad Leader',
            'freeze_leader': 'Freeze Ldr',
            'self_injure_leader': 'Self-Injure',
        }.get(fault, fault)
        
        print(f"{ctrl.upper():<8} {fault_display:<15} {n:>3}  {avg_cov:>9.1f}%  {avg_speed:>7.0f}")

if __name__ == '__main__':
    main()
