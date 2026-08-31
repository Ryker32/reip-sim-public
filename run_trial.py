#!/usr/bin/env python3
"""
Fully automated hardware trial runner.

Handles everything: deploy, launch, record video, inject faults on
schedule, stop robots, collect logs. Zero human intervention.

Usage:
  python run_trial.py                          # REIP + bad_leader (default)
  python run_trial.py --controller reip --fault bad_leader
  python run_trial.py --controller reip --fault self_injure_leader
  python run_trial.py --controller reip --fault freeze_leader
  python run_trial.py --controller raft --fault bad_leader
  python run_trial.py --controller reip --fault none          # clean run
  python run_trial.py --controller decentralized --fault none
  python run_trial.py --robots 1              # single-robot test (only deploy/launch R1)
  python run_trial.py --batch                  # run full experiment matrix

Timing (hardware paper-trial mode by default):
  t=0s   : deploy + launch robots
  t=10s  : FAULT #1 on current leader
  t=30s  : FAULT #2 on current leader (may differ after impeachment)
  t=120s : stop robots, collect logs

Why startup takes a while: kill+clear (parallel), then SSH upload to all robots
(parallel, timeout ~10s), then launch + 1s pause + parallel verify. Finally
START_DELAY_SEC (default 5s) for localization + leader election. Use
--start-delay N to change that last wait.

Position server (aruco_position_server.py) must be running separately.
Press 'r' in its window to start/stop video recording, OR pass
--no-video to skip manual recording.
"""

import argparse
import concurrent.futures
import json
import os
import socket
import subprocess
import sys
import time
from datetime import datetime

import paramiko

# Cap for any single SSH/SFTP operation so we never hang indefinitely on flaky WiFi
SSH_OP_TIMEOUT = 15

# ==================== Config (must match other scripts) ====================
HOSTS = {
    1: '192.168.20.16',
    2: '192.168.20.15',
    3: '192.168.20.112',
    4: '192.168.20.22',
    5: '192.168.20.18',
}
PASSWORD = os.environ.get("REIP_ROBOT_SSH_PASSWORD")
NUM_ROBOTS = 5

UDP_PEER_PORT = 5200
UDP_FAULT_PORT = 5300
BROADCAST_IP = "192.168.20.255"
WIFI_BIND_IP = "192.168.20.214"

EXPERIMENT_DURATION = 120
FAULT_INJECT_TIME_1 = 20
FAULT_INJECT_TIME_2 = 40
# Time from "go" to trial clock start. Robots need boot + localization + leader election.
# Reduce to 5s for fast networks; increase if Pis are slow to boot or election is flaky.
START_DELAY_SEC = 5

REIP_LOCAL = os.path.join(os.path.dirname(__file__), 'robot', 'reip_node.py')
RAFT_LOCAL = os.path.join(os.path.dirname(__file__), 'robot', 'baselines', 'raft_node.py')

# Full hardware experiment matrix: one fault condition per run
EXPERIMENT_MATRIX = [
    # (controller, fault, description)
    ("reip",          "none",         "REIP clean baseline"),
    ("reip",          "bad_leader",   "REIP vs bad leader (KEY DEMO)"),
    ("reip",          "freeze_leader", "REIP vs freeze leader"),
    ("reip",          "self_injure_leader", "REIP vs follower crash"),
    ("raft",          "none",         "Raft clean baseline"),
    ("raft",          "bad_leader",   "Raft vs bad leader (no detection)"),
    ("raft",          "freeze_leader", "Raft vs freeze leader"),
    ("raft",          "self_injure_leader", "Raft vs follower crash"),
]
TRIALS_PER_CONDITION = 3


# ==================== SSH helpers ====================
# Per-operation SSH timeout -- 2s was too aggressive for Pi Zero 2W under load
SSH_TIMEOUT = 8


def check_connectivity(robot_ids=None):
    """Ping each robot IP once; print which are reachable. Use before deploy when WiFi is flaky."""
    if robot_ids is None:
        robot_ids = sorted(HOSTS.keys())
    print("[PREFLIGHT] Checking connectivity...")
    for rid in robot_ids:
        host = HOSTS[rid]
        try:
            # Windows: -n 1, -w 2000ms; Linux/Mac: -c 1, -W 2
            out = subprocess.run(
                ['ping', '-n', '1', '-w', '2000', host] if sys.platform == 'win32' else ['ping', '-c', '1', '-W', '2', host],
                capture_output=True, timeout=5)
            ok = out.returncode == 0
            print(f"  R{rid} ({host}): {'OK' if ok else 'no reply'}")
        except Exception as e:
            print(f"  R{rid} ({host}): {e}")

def _ssh_connect(host, timeout=None):
    if timeout is None:
        timeout = SSH_TIMEOUT
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(host, username='pi', password=PASSWORD, timeout=timeout)
    # Fix: bound every subsequent op (sftp put/get, exec read) so nothing hangs
    t = ssh.get_transport()
    if t and t.sock:
        t.sock.settimeout(SSH_OP_TIMEOUT)
    return ssh


def _kill_one(rid, host):
    try:
        ssh = _ssh_connect(host)
        ssh.exec_command(
            'pkill -INT -f reip_node; pkill -INT -f raft_node; sleep 1; '
            'pkill -9 -f reip_node; pkill -9 -f raft_node; pkill -9 -f "python3.*node.py"'
        )
        ssh.close()
        print(f"  R{rid}: killed")
    except Exception as e:
        print(f"  R{rid}: kill failed ({e})")


def kill_all(robot_ids=None):
    if robot_ids is None:
        robot_ids = sorted(HOSTS.keys())
    print(f"[KILL] Stopping robots: {robot_ids}")
    import concurrent.futures
    with concurrent.futures.ThreadPoolExecutor(max_workers=5) as ex:
        futs = {
            ex.submit(_kill_one, rid, HOSTS[rid]): rid
            for rid in robot_ids
        }
        concurrent.futures.wait(futs, timeout=SSH_TIMEOUT + 5)
    time.sleep(0.5)


def clear_robot_logs(robot_ids=None):
    """Remove old logs so collect_logs only gets this trial's data."""
    if robot_ids is None:
        robot_ids = sorted(HOSTS.keys())
    import concurrent.futures
    def _clear_one(rid, host):
        try:
            ssh = _ssh_connect(host)
            ssh.exec_command('rm -f /home/pi/reip/logs/*.jsonl')
            ssh.close()
        except Exception:
            pass
    with concurrent.futures.ThreadPoolExecutor(max_workers=5) as ex:
        futs = [ex.submit(_clear_one, rid, HOSTS[rid]) for rid in robot_ids]
        concurrent.futures.wait(futs, timeout=SSH_TIMEOUT * len(robot_ids) + 2)


def deploy_and_launch(controller, robot_ids=None):
    """Two-phase deploy: upload code, then launch with stagger.

    Returns the list of robot IDs confirmed running.
    Calls sys.exit(1) if zero robots start.
    """
    if robot_ids is None:
        robot_ids = sorted(HOSTS.keys())
    if controller == 'raft':
        local_file = RAFT_LOCAL
        remote_script = 'baselines/raft_node.py'
    else:
        local_file = REIP_LOCAL
        remote_script = 'reip_node.py'

    decentralized_flag = ' --decentralized' if controller == 'decentralized' else ''

    # --- Phase 1: Kill old processes + upload code (parallel, fail fast) ---
    print(f"[DEPLOY] Phase 1: uploading {remote_script} to robot(s) {robot_ids}...")
    def _upload_one(rid, host):
        try:
            ssh = _ssh_connect(host)
            ssh.exec_command('pkill -f reip_node; pkill -f raft_node; sleep 0.5; pkill -9 -f reip_node; pkill -9 -f raft_node; pkill -9 -f "python3.*node.py"')
            time.sleep(1.0)
            sftp = ssh.open_sftp()
            for d in ['/home/pi/reip', '/home/pi/reip/logs', '/home/pi/reip/baselines']:
                try:
                    sftp.mkdir(d)
                except Exception:
                    pass
            sftp.put(local_file, f'/home/pi/reip/{remote_script}')
            sftp.close()
            return (rid, ssh)
        except Exception as e:
            print(f"  R{rid}: ERROR - {e}")
            return (rid, None)

    ssh_connections = {}
    upload_timeout = max(20, SSH_TIMEOUT * len(robot_ids) + 5)
    with concurrent.futures.ThreadPoolExecutor(max_workers=5) as ex:
        futs = {ex.submit(_upload_one, rid, HOSTS[rid]): rid for rid in robot_ids}
        try:
            for fut in concurrent.futures.as_completed(futs, timeout=upload_timeout):
                rid, ssh = fut.result()
                if ssh is not None:
                    ssh_connections[rid] = ssh
                    print(f"  R{rid}: uploaded")
        except concurrent.futures.TimeoutError:
            pass
    missing = [r for r in robot_ids if r not in ssh_connections]
    if missing:
        print(f"[DEPLOY] FAILED: robot(s) {missing} did not upload. Fix network or run with fewer robots.")
        sys.exit(1)

    time.sleep(0.5)

    # --- Phase 2: Launch with stagger (0.4s gap avoids WiFi socket storm) ---
    print(f"[DEPLOY] Phase 2: launching robots (staggered)...")
    launch_cmds = {}
    for rid, ssh in ssh_connections.items():
        cmd = (f'cd /home/pi/reip && nohup python3 {remote_script} {rid}'
               f'{decentralized_flag}'
               f' > /tmp/reip_{rid}.log 2>&1 &')
        launch_cmds[rid] = cmd
        for attempt in range(2):
            try:
                ssh.exec_command(cmd)
                break
            except Exception as e:
                if attempt == 0:
                    print(f"  R{rid}: launch error ({e}), reconnecting...")
                    try:
                        ssh.close()
                    except Exception:
                        pass
                    try:
                        ssh2 = _ssh_connect(HOSTS[rid])
                        ssh_connections[rid] = ssh2
                        ssh = ssh2
                    except Exception as e2:
                        print(f"  R{rid}: reconnect FAILED - {e2}")
                        break
                else:
                    print(f"  R{rid}: relaunch FAILED - {e}")
        time.sleep(0.4)

    time.sleep(2)

    # --- Phase 3: Verify which robots are actually running ---
    running = []
    failed_launch = []
    def _verify_one(rid):
        try:
            ssh = _ssh_connect(HOSTS[rid])
            _, stdout, _ = ssh.exec_command(f'pgrep -f "{remote_script}"')
            pid = stdout.read().decode().strip()
            ssh.close()
            return (rid, pid)
        except Exception as e:
            return (rid, None)
    with concurrent.futures.ThreadPoolExecutor(max_workers=5) as ex:
        vfuts = {ex.submit(_verify_one, rid): rid for rid in ssh_connections}
        for fut in concurrent.futures.as_completed(vfuts, timeout=upload_timeout):
            rid, pid = fut.result()
            if pid:
                running.append(rid)
                print(f"  R{rid}: RUNNING (pid={pid})")
            else:
                failed_launch.append(rid)
                print(f"  R{rid}: FAILED")

    # Close any leftover SSH connections from Phase 2
    for ssh in ssh_connections.values():
        try:
            ssh.close()
        except Exception:
            pass

    running.sort()
    if not running:
        print(f"\n[DEPLOY] FATAL: No robots started! Aborting.")
        sys.exit(1)
    elif failed_launch:
        print(f"\n[DEPLOY] WARNING: {len(failed_launch)} robot(s) failed: {failed_launch}")
        print(f"[DEPLOY] Continuing with {len(running)} robot(s): {running}")

    return running


def _collect_logs_one(rid, host, trial_dir):
    """Fetch logs from one robot; can block on slow SFTP so caller must use timeout."""
    ssh = _ssh_connect(host)
    sftp = ssh.open_sftp()
    try:
        for f in sftp.listdir('/home/pi/reip/logs'):
            if f.endswith('.jsonl'):
                sftp.get(f'/home/pi/reip/logs/{f}',
                         os.path.join(trial_dir, f'r{rid}_{f}'))
                print(f"  R{rid}: {f}")
    except FileNotFoundError:
        print(f"  R{rid}: no logs")
    finally:
        sftp.close()
        ssh.close()


def collect_logs(trial_dir, robot_ids=None):
    if robot_ids is None:
        robot_ids = sorted(HOSTS.keys())
    os.makedirs(trial_dir, exist_ok=True)
    print(f"[LOGS] Collecting to {trial_dir}")
    failures = []
    for rid in robot_ids:
        host = HOSTS[rid]
        try:
            _collect_logs_one(rid, host, trial_dir)
        except Exception as e:
            print(f"  R{rid}: {e}")
            failures.append(rid)
    if failures:
        failed_str = ", ".join(f"R{rid}" for rid in failures)
        print(f"[LOGS] WARNING: could not collect from {failed_str}.")
        return False
    return True


# ==================== Fault injection ====================
def find_leader(timeout=4.0):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((WIFI_BIND_IP, UDP_PEER_PORT))
    sock.settimeout(0.2)

    votes = {}
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            data, _ = sock.recvfrom(65536)
            msg = json.loads(data.decode())
            if msg.get('type') == 'peer_state':
                lid = msg.get('leader_id')
                if lid:
                    votes[lid] = votes.get(lid, 0) + 1
        except socket.timeout:
            pass
    sock.close()
    return max(votes, key=votes.get) if votes else -1


def inject_fault(robot_id, fault_type, extra=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind((WIFI_BIND_IP, 0))
    payload = {
        'type': 'fault_inject',
        'robot_id': robot_id,
        'fault': fault_type,
        'timestamp': time.time()
    }
    if extra:
        payload.update(extra)
    msg = json.dumps(payload).encode()
    sock.sendto(msg, (BROADCAST_IP, UDP_FAULT_PORT))
    sock.close()


def clear_faults(robot_ids):
    for rid in robot_ids:
        inject_fault(rid, 'none')


# ==================== Single trial ====================
def run_single_trial(controller, fault_type, trial_num, output_dir, robot_ids=None,
                    start_delay=None, preflight=False, fault_time_1=FAULT_INJECT_TIME_1,
                    fault_time_2=FAULT_INJECT_TIME_2):
    if robot_ids is None:
        robot_ids = sorted(HOSTS.keys())
    if preflight:
        check_connectivity(robot_ids)
        print()
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    trial_name = f"{controller}_{fault_type or 'none'}_t{trial_num}"
    trial_dir = os.path.join(output_dir, f"{trial_name}_{ts}")
    os.makedirs(trial_dir, exist_ok=True)

    print(f"\n{'='*60}")
    print(f" TRIAL: {trial_name}")
    print(f" Controller: {controller}")
    print(f" Fault: {fault_type or 'none'}")
    print(f" Robots: {robot_ids}")
    print(f" Duration: {EXPERIMENT_DURATION}s")
    if fault_type and fault_type != 'none':
        print(f" Fault at t={fault_time_1}s")
        if fault_time_2 is not None:
            print(f" Fault #2 at t={fault_time_2}s")
    print(f" Output: {trial_dir}")
    print(f"{'='*60}\n")

    # Save trial metadata
    meta = {
        'trial_name': trial_name, 'controller': controller,
        'fault_type': fault_type, 'trial_num': trial_num,
        'duration': EXPERIMENT_DURATION,
        'fault_time_1': fault_time_1 if fault_type and fault_type != 'none' else None,
        'fault_time_2': fault_time_2 if fault_type and fault_type != 'none' else None,
        'start_time': time.time(), 'start_ts': ts,
    }

    # Phase 0: Kill old processes + clear old logs
    kill_all(robot_ids)
    clear_robot_logs(robot_ids)

    # Phase 1: Deploy and launch -- returns only robots confirmed running
    active_robots = deploy_and_launch(controller, robot_ids)

    if len(active_robots) < len(robot_ids):
        print(f"\n[WARNING] Only {len(active_robots)}/{len(robot_ids)} robots running.")
        print(f"          Requested: {robot_ids}  |  Active: {active_robots}")
    meta['active_robots'] = active_robots

    print(f"\n[CONFIG] Broadcasting active robot set: {active_robots}")
    trial_config = {
        'active_robot_ids': list(active_robots),
        'active_robot_count': len(active_robots),
    }
    for _ in range(3):
        inject_fault(0, 'set_trial_robots', trial_config)
        time.sleep(0.1)

    # Give robots time to get localized and elect a leader, then send start signal.
    # (Startup: kill+clear ~2--4s, deploy Phase1 up to ~10s, Phase2+verify ~2--3s, then this delay.)
    delay = start_delay if start_delay is not None else START_DELAY_SEC
    print(f"\n[WAIT] Giving robots {delay}s for localization + leader election...")
    time.sleep(delay)

    # Reset coverage on the position server overlay
    _reset_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    _reset_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    _reset_sock.sendto(json.dumps({'type': 'reset_coverage'}).encode(),
                       (BROADCAST_IP, UDP_PEER_PORT))
    _reset_sock.close()

    print(f"[START] Sending start signal to all robots...")
    for _ in range(10):
        inject_fault(0, 'start', trial_config)
        time.sleep(0.15)

    t0 = time.time()
    fault_robots = []

    print(f"\n[t=0.0s] Trial clock started.")
    print(f"         Position server should be running + recording.\n")

    try:
        while True:
            elapsed = time.time() - t0

            if elapsed >= EXPERIMENT_DURATION:
                break

            # Fault #1
            if (fault_type and fault_type != 'none'
                    and elapsed >= fault_time_1
                    and len(fault_robots) == 0):
                print(f"[t={elapsed:.1f}s] Scanning for leader...")
                leader = find_leader(timeout=3.0)
                if leader > 0:
                    inject_fault(leader, fault_type)
                    fault_robots.append(leader)
                    meta['fault1_robot'] = leader
                    meta['fault1_actual_time'] = elapsed
                    print(f"[t={time.time()-t0:.1f}s] FAULT #1: {fault_type} on Robot {leader}")
                else:
                    print(f"[t={elapsed:.1f}s] WARNING: no leader found for fault #1")
                    fault_robots.append(None)

            # Fault #2
            if (fault_type and fault_type != 'none'
                    and fault_time_2 is not None
                    and elapsed >= fault_time_2
                    and len(fault_robots) == 1):
                print(f"\n[t={elapsed:.1f}s] Scanning for leader (fault #2)...")
                leader2 = find_leader(timeout=3.0)
                if leader2 > 0:
                    if leader2 != fault_robots[0]:
                        print(f"  New leader after impeachment: Robot {leader2}")
                    else:
                        print(f"  Same leader: Robot {leader2} (re-injecting)")
                    inject_fault(leader2, fault_type)
                    fault_robots.append(leader2)
                    meta['fault2_robot'] = leader2
                    meta['fault2_actual_time'] = elapsed
                    print(f"[t={time.time()-t0:.1f}s] FAULT #2: {fault_type} on Robot {leader2}")
                else:
                    print(f"[t={elapsed:.1f}s] WARNING: no leader found for fault #2")
                    fault_robots.append(None)

            # Progress report every 15s
            if int(elapsed) % 15 == 0 and int(elapsed) > 0:
                remaining = EXPERIMENT_DURATION - elapsed
                if abs(elapsed - int(elapsed)) < 0.5:
                    print(f"[t={elapsed:.0f}s] {remaining:.0f}s remaining...")

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\n[!] Trial interrupted by user.")

    # Phase 3: Stop
    elapsed_final = time.time() - t0
    print(f"\n[t={elapsed_final:.1f}s] Trial complete.")
    meta['actual_duration'] = elapsed_final

    clear_faults([r for r in fault_robots if r])
    time.sleep(0.5)
    kill_all(robot_ids)

    # Phase 4: Collect logs
    time.sleep(2)
    collect_logs(trial_dir, robot_ids)

    # Save metadata
    meta_path = os.path.join(trial_dir, 'trial_meta.json')
    with open(meta_path, 'w') as f:
        json.dump(meta, f, indent=2)
    print(f"\n[DONE] Trial data in {trial_dir}\n")
    return trial_dir


# ==================== Batch mode ====================
def run_batch(output_dir, trials_per=TRIALS_PER_CONDITION, start_delay=None, preflight=False):
    total = len(EXPERIMENT_MATRIX) * trials_per
    print(f"\n{'#'*60}")
    print(f"  BATCH MODE: {len(EXPERIMENT_MATRIX)} conditions x {trials_per} trials = {total} runs")
    print(f"  Estimated time: {total * (EXPERIMENT_DURATION + 30) / 60:.0f} minutes")
    print(f"{'#'*60}\n")

    if preflight:
        check_connectivity()
        print()
    input("Place robots in arena and press ENTER to begin...")

    completed = 0
    for trial_num in range(1, trials_per + 1):
        for controller, fault, desc in EXPERIMENT_MATRIX:
            completed += 1
            print(f"\n{'*'*60}")
            print(f"  [{completed}/{total}] {desc}  (trial {trial_num})")
            print(f"{'*'*60}")

            run_single_trial(controller, fault, trial_num, output_dir, start_delay=start_delay)

            if completed < total:
                print("\n[PAUSE] Reposition robots in arena.")
                input("Press ENTER when ready for next trial...")

    print(f"\n{'#'*60}")
    print(f"  ALL {total} TRIALS COMPLETE")
    print(f"  Data in {output_dir}")
    print(f"{'#'*60}")


# ==================== Entry ====================
if __name__ == '__main__':
    def _parse_robot_ids(spec: str):
        ids = []
        for chunk in spec.split(','):
            chunk = chunk.strip()
            if not chunk:
                continue
            rid = int(chunk)
            if rid not in HOSTS:
                raise ValueError(f"Unknown robot id: {rid}")
            ids.append(rid)
        if not ids:
            raise ValueError("No robot ids provided")
        return sorted(set(ids))

    parser = argparse.ArgumentParser(description="Automated hardware trial runner")
    parser.add_argument('--controller', default='reip',
                        choices=['reip', 'raft', 'decentralized'])
    parser.add_argument('--fault', default='bad_leader',
                        help='Fault type: bad_leader, self_injure_leader, freeze_leader, oscillate_leader, spin, none')
    parser.add_argument('--trial', type=int, default=1, help='Trial number')
    parser.add_argument('--output', default='trials', help='Output directory')
    parser.add_argument('--batch', action='store_true',
                        help='Run full experiment matrix')
    parser.add_argument('--trials-per', type=int, default=TRIALS_PER_CONDITION,
                        help='Trials per condition in batch mode')
    parser.add_argument('--duration', type=int, default=EXPERIMENT_DURATION,
                        help='Trial duration in seconds')
    parser.add_argument('--fault-time', type=float, default=FAULT_INJECT_TIME_1,
                        help=f'Seconds after trial start for the primary fault injection (default: {FAULT_INJECT_TIME_1})')
    parser.add_argument('--second-fault-time', type=float, default=FAULT_INJECT_TIME_2,
                        help=f'Seconds after trial start for the second fault injection (default: {FAULT_INJECT_TIME_2}). Use a negative value to disable.')
    parser.add_argument('--start-delay', type=float, default=None,
                        help=f'Seconds to wait after launch before trial start (default: {START_DELAY_SEC})')
    parser.add_argument('--preflight', action='store_true',
                        help='Ping all robot IPs before deploy (see who is reachable)')
    parser.add_argument('--robots', default='1,2,3,4,5',
                        help='Robot(s) to deploy and run. Use --robots 1 for single-robot testing when only R1 is reachable.')
    args = parser.parse_args()

    EXPERIMENT_DURATION = args.duration
    robot_ids = _parse_robot_ids(args.robots)
    second_fault_time = None if args.second_fault_time is not None and args.second_fault_time < 0 else args.second_fault_time

    if args.batch:
        run_batch(args.output, args.trials_per, start_delay=args.start_delay, preflight=args.preflight)
    else:
        run_single_trial(args.controller, args.fault, args.trial, args.output, robot_ids,
                        start_delay=args.start_delay, preflight=args.preflight,
                        fault_time_1=args.fault_time, fault_time_2=second_fault_time)
