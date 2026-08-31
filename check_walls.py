import paramiko
import json
import os

# SSH password for the robot Pis. Set REIP_ROBOT_SSH_PASSWORD in your environment.
SSH_PASSWORD = os.environ.get("REIP_ROBOT_SSH_PASSWORD")

for rid in [1, 2]:
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(f'clanker{rid}.local', username='pi', password=SSH_PASSWORD, timeout=10)
    
    print(f"=== Last 5 log entries (robot {rid}) ===")
    stdin, stdout, stderr = ssh.exec_command(f'tail -5 ~/reip/logs/robot_{rid}_*.jsonl')
    out = stdout.read().decode().strip()
    for line in out.split('\n'):
        line = line.strip()
        if not line:
            continue
        try:
            d = json.loads(line)
            print(f"  pos=({d['x']:.0f},{d['y']:.0f}) theta={d['theta']:.2f} state={d['state']} leader={d['leader_id']} target={d['commanded_target']} visited={d['known_visited_count']}")
        except:
            print(f"  (parse error: {line[:80]})")
    
    ssh.close()
