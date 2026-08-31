#!/usr/bin/env python3
"""Parse a hardware trial directory and print a human-readable summary."""
import json, os, sys, glob
from collections import defaultdict

def parse_trial(trial_dir):
    meta_path = os.path.join(trial_dir, 'trial_meta.json')
    if os.path.exists(meta_path):
        with open(meta_path) as f:
            meta = json.load(f)
        print(f"=== Trial: {meta.get('trial_name')} ===")
        print(f"  Controller: {meta.get('controller')}")
        print(f"  Fault: {meta.get('fault_type', 'none')}")
        print(f"  Duration: {meta.get('actual_duration', 0):.1f}s")
        print(f"  Active robots: {meta.get('active_robots')}")
        if meta.get('fault1_robot'):
            print(f"  Fault #1: on R{meta['fault1_robot']} at t={meta.get('fault1_actual_time', '?'):.1f}s")
        if meta.get('fault2_robot'):
            print(f"  Fault #2: on R{meta['fault2_robot']} at t={meta.get('fault2_actual_time', '?'):.1f}s")
        print()

    logs = sorted(glob.glob(os.path.join(trial_dir, 'r*_robot_*.jsonl')))
    if not logs:
        print("  No robot logs found!")
        return

    all_data = {}
    for logfile in logs:
        rid = int(os.path.basename(logfile).split('_')[0][1:])
        entries = []
        with open(logfile) as f:
            for line in f:
                line = line.strip()
                if line:
                    entries.append(json.loads(line))
        all_data[rid] = entries

    t0 = min(e[0]['t'] for e in all_data.values() if e)

    for rid in sorted(all_data.keys()):
        entries = all_data[rid]
        if not entries:
            print(f"  R{rid}: empty log")
            continue

        first = entries[0]
        last = entries[-1]
        duration = last['t'] - first['t']

        states = [e['state'] for e in entries]
        leader_ids = [e.get('leader_id') for e in entries if e.get('leader_id')]
        was_leader = any(e['state'] == 'leader' for e in entries)

        started_moving = None
        for e in entries:
            if e.get('stop_reason') != 'trial_not_started' and e.get('motor_cmd', [0, 0]) != [0, 0]:
                started_moving = e['t'] - t0
                break

        final_visited = last.get('my_visited_count', 0)
        known_visited = last.get('known_visited_count', 0)
        final_trust = last.get('trust_in_leader', 1.0)
        final_suspicion = last.get('suspicion', 0.0)

        impeachment_t = None
        for e in entries:
            if e.get('impeachment_time') and impeachment_t is None:
                impeachment_t = e['impeachment_time'] - t0

        peer_em = last.get('peer_emergency_debug', {})
        peer_em_count = peer_em.get('count', 0)
        peer_em_hard = peer_em.get('hard_count', 0)

        leader_changes = []
        prev_lid = None
        for e in entries:
            lid = e.get('leader_id')
            if lid != prev_lid and lid is not None:
                leader_changes.append((e['t'] - t0, lid))
                prev_lid = lid

        print(f"  R{rid}: {len(entries)} ticks, {duration:.1f}s")
        print(f"    Position: ({last['x']:.0f}, {last['y']:.0f})  theta={last['theta']:.2f}")
        print(f"    Final state: {last['state']}, leader=R{last.get('leader_id', '?')}")
        print(f"    Visited: {final_visited} (self), {known_visited} (known global)")
        if was_leader:
            print(f"    ** Was leader at some point **")
        print(f"    Trust: {final_trust:.3f}, Suspicion: {final_suspicion:.3f}")
        if started_moving is not None:
            print(f"    Started moving at t={started_moving:.1f}s")
        else:
            print(f"    NEVER MOVED (motors stayed [0,0])")
        if impeachment_t is not None:
            print(f"    Impeachment triggered at t={impeachment_t:.1f}s")
        print(f"    Peer emergencies: {peer_em_count} (hard: {peer_em_hard})")
        if len(leader_changes) > 1:
            print(f"    Leader changes: {len(leader_changes)}")
            for t_change, lid in leader_changes[:5]:
                print(f"      t={t_change:.1f}s -> R{lid}")
            if len(leader_changes) > 5:
                print(f"      ... +{len(leader_changes)-5} more")
        print()

    # Coverage estimate: union of visited cells across all robots
    print("--- Coverage Summary ---")
    final_known = max(e[-1].get('known_visited_count', 0) for e in all_data.values())
    from arena_coverage import TOTAL_REACHABLE_CELLS
    total_cells = TOTAL_REACHABLE_CELLS  # derived from DEFAULT_ARENA.is_wall_cell()
    coverage_pct = final_known / total_cells * 100
    print(f"  Known visited cells: {final_known}/{total_cells} = {coverage_pct:.1f}%")

    # Leader timeline
    print("\n--- Leader Timeline ---")
    leader_events = []
    for rid in sorted(all_data.keys()):
        prev_state = None
        for e in all_data[rid]:
            if e['state'] != prev_state:
                leader_events.append((e['t'] - t0, rid, e['state'], e.get('leader_id')))
                prev_state = e['state']
    leader_events.sort()
    for t_ev, rid, state, lid in leader_events:
        if state == 'leader':
            print(f"  t={t_ev:6.1f}s  R{rid} became LEADER")
        elif 'election' in state.lower() if state else False:
            print(f"  t={t_ev:6.1f}s  R{rid} entered {state}")

    # Unique leaders
    all_leaders = set()
    for rid in sorted(all_data.keys()):
        for e in all_data[rid]:
            if e['state'] == 'leader':
                all_leaders.add(rid)
    print(f"\n  Robots that served as leader: {sorted(all_leaders) if all_leaders else 'none'}")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        dirs = sorted(glob.glob('trials/*/'))
        if dirs:
            trial_dir = dirs[-1]
            print(f"(Using most recent: {trial_dir})\n")
        else:
            print("Usage: python parse_trial.py <trial_dir>")
            sys.exit(1)
    else:
        trial_dir = sys.argv[1]
    parse_trial(trial_dir)
