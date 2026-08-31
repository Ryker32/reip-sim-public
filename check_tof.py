import paramiko, json
import os

# SSH password for the robot Pis. Set REIP_ROBOT_SSH_PASSWORD in your environment.
SSH_PASSWORD = os.environ.get("REIP_ROBOT_SSH_PASSWORD")

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('clanker1.local', username='pi', password=SSH_PASSWORD, timeout=10)

# Find latest log file
stdin, stdout, stderr = ssh.exec_command('ls -t ~/reip/logs/robot_1_*.jsonl 2>/dev/null | head -1')
logfile = stdout.read().decode().strip()
print(f"Log file: {logfile}")

if logfile:
    print("\n=== Last 3 log entries ===")
    stdin, stdout, stderr = ssh.exec_command(f'tail -3 {logfile}')
    out = stdout.read().decode().strip()
    for line in out.split('\n'):
        if not line.strip():
            continue
        try:
            d = json.loads(line)
            print(f"  pos=({d['x']:.0f},{d['y']:.0f}) state={d['state']} target={d['commanded_target']} visited={d['known_visited_count']}")
        except:
            print(f"  (parse err)")

print("\n=== ToF sensor quick test ===")
stdin, stdout, stderr = ssh.exec_command(
    'python3 -u -c "'
    'import sys; sys.path.insert(0, chr(47)+chr(104)+chr(111)+chr(109)+chr(101)+chr(47)+chr(112)+chr(105)+chr(47)+chr(114)+chr(101)+chr(105)+chr(112)); '
    'from reip_node import Hardware; '
    'hw = Hardware(); '
    'tof = hw.read_tof_all(); '
    'print(tof)" 2>&1'
)
print(stdout.read().decode().strip())

ssh.close()
