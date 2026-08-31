"""Deploy updated reip_node.py to R1 and test import."""
import paramiko
import os

# SSH password for the robot Pis. Set REIP_ROBOT_SSH_PASSWORD in your environment.
SSH_PASSWORD = os.environ.get("REIP_ROBOT_SSH_PASSWORD")

LOCAL = os.path.join(os.path.dirname(__file__), 'robot', 'reip_node.py')

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('192.168.20.16', username='pi', password=SSH_PASSWORD, timeout=10)

print("Uploading fixed reip_node.py...")
sftp = ssh.open_sftp()
sftp.put(LOCAL, '/home/pi/reip/reip_node.py')
sftp.close()
print("  Done.")

print("\nTesting import on Pi...")
_, stdout, _ = ssh.exec_command(
    'cd /home/pi/reip && python3 -c "'
    'import reip_node; '
    'print(reip_node.ARENA_WIDTH, reip_node.ARENA_HEIGHT, reip_node.CELL_SIZE); '
    'print(reip_node.DEFAULT_ARENA.is_wall_cell(0, 0)); '
    'print(type(reip_node.DEFAULT_ARENA))'
    '" 2>&1')
print(stdout.read().decode())

print("Testing launch + immediate pgrep...")
_, stdout, _ = ssh.exec_command(
    'cd /home/pi/reip && nohup python3 reip_node.py 1 > /tmp/reip_1.log 2>&1 & '
    'sleep 2 && pgrep -f reip_node')
pid = stdout.read().decode().strip()
print(f"  PID: {pid}" if pid else "  NOT RUNNING")

if pid:
    print("\nChecking /tmp/reip_1.log (first 10 lines)...")
    _, stdout, _ = ssh.exec_command('head -10 /tmp/reip_1.log')
    print(stdout.read().decode())
    ssh.exec_command('pkill -f reip_node')
else:
    print("\nCrash log:")
    _, stdout, _ = ssh.exec_command('cat /tmp/reip_1.log')
    print(stdout.read().decode())

ssh.close()
