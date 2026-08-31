import paramiko, json
import os

# SSH password for the robot Pis. Set REIP_ROBOT_SSH_PASSWORD in your environment.
SSH_PASSWORD = os.environ.get("REIP_ROBOT_SSH_PASSWORD")

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('192.168.20.22', username='pi', password=SSH_PASSWORD, timeout=5)

# Get latest JSONL log
stdin, stdout, stderr = ssh.exec_command(
    'ls -t ~/reip/logs/robot_4_*.jsonl 2>/dev/null | head -1')
logfile = stdout.read().decode().strip()
print(f"Log file: {logfile}")

if logfile:
    stdin, stdout, stderr = ssh.exec_command(f'tail -5 {logfile}')
    for line in stdout.read().decode().strip().split('\n'):
        if line.strip():
            d = json.loads(line)
            print(f"  pos=({d.get('x',0):.0f},{d.get('y',0):.0f}) "
                  f"state={d.get('state')} "
                  f"nav={d.get('navigation_target')} "
                  f"pred={d.get('predicted_target')} "
                  f"leader_self={d.get('leader_self_target')} "
                  f"fault={d.get('fault')} "
                  f"my_vis={d.get('my_visited_count')} "
                  f"known_vis={d.get('known_visited_count')}")

# Also get full stderr log to see the trust impeachment sequence
stdin, stdout, stderr = ssh.exec_command('tail -30 /tmp/reip_4.log')
print(f"\nFull log tail:")
print(stdout.read().decode().strip())

ssh.close()
