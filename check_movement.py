import paramiko, time
import os

# SSH password for the robot Pis. Set REIP_ROBOT_SSH_PASSWORD in your environment.
SSH_PASSWORD = os.environ.get("REIP_ROBOT_SSH_PASSWORD")

HOSTS = {
    1: '192.168.20.16',
    2: '192.168.20.15',
    3: '192.168.20.112',
    4: '192.168.20.22',
    5: '192.168.20.18',
}

def get_positions():
    positions = {}
    for rid in sorted(HOSTS.keys()):
        try:
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(HOSTS[rid], username='pi', password=SSH_PASSWORD, timeout=5)
            stdin, stdout, stderr = ssh.exec_command(f'tail -1 /tmp/reip_{rid}.log')
            line = stdout.read().decode().strip()
            positions[rid] = line
            ssh.close()
        except:
            positions[rid] = "ERROR"
    return positions

print("=== Snapshot 1 ===")
p1 = get_positions()
for rid, line in p1.items():
    print(f"  R{rid}: {line}")

print("\nWaiting 5 seconds...\n")
time.sleep(5)

print("=== Snapshot 2 ===")
p2 = get_positions()
for rid, line in p2.items():
    print(f"  R{rid}: {line}")
    if p1[rid] == p2[rid]:
        print(f"         ^ SAME (not moving)")
    else:
        print(f"         ^ CHANGED (moving)")
