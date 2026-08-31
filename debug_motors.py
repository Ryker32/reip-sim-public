import paramiko, json, time
import os

# SSH password for the robot Pis. Set REIP_ROBOT_SSH_PASSWORD in your environment.
SSH_PASSWORD = os.environ.get("REIP_ROBOT_SSH_PASSWORD")

HOSTS = {1:'192.168.20.16', 2:'192.168.20.15', 3:'192.168.20.112'}

for rid in [1, 2]:
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(HOSTS[rid], username='pi', password=SSH_PASSWORD, timeout=5)

        # Get the latest JSONL log from NEW run
        stdin, stdout, stderr = ssh.exec_command(
            f'ls -t ~/reip/logs/robot_{rid}_*.jsonl 2>/dev/null | head -1')
        logfile = stdout.read().decode().strip()
        
        if logfile:
            # Get last 5 JSONL entries
            stdin, stdout, stderr = ssh.exec_command(f'tail -5 {logfile}')
            lines = stdout.read().decode().strip().split('\n')
            print(f"=== R{rid} latest JSONL ({logfile}) ===")
            for line in lines:
                if line.strip():
                    d = json.loads(line)
                    print(f"  pos=({d.get('x',0):.0f},{d.get('y',0):.0f}) "
                          f"state={d.get('state')} "
                          f"nav={d.get('navigation_target')} "
                          f"pred={d.get('predicted_target')} "
                          f"cmd={d.get('commanded_target')} "
                          f"leader_self={d.get('leader_self_target')} "
                          f"visited={d.get('known_visited_count')}")
        else:
            print(f"=== R{rid}: no JSONL log ===")

        # Full stderr last 10 lines
        stdin, stdout, stderr = ssh.exec_command(f'tail -10 /tmp/reip_{rid}.log')
        log = stdout.read().decode().strip()
        print(f"  stderr: {log}")
        print()
        ssh.close()
    except Exception as e:
        print(f"R{rid}: {e}")
