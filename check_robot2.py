import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('clanker1.local', username='pi', password=SSH_PASSWORD, timeout=10)

print("=== Last 30 lines of JSONL log ===")
stdin, stdout, stderr = ssh.exec_command('tail -30 ~/reip/logs/robot_1_*.jsonl 2>&1')
print(stdout.read().decode())

print("=== Is reip_node receiving positions? ===")
stdin, stdout, stderr = ssh.exec_command('grep -c "position" ~/reip/logs/robot_1_*.jsonl 2>&1')
print(f"Position entries: {stdout.read().decode().strip()}")

stdin, stdout, stderr = ssh.exec_command('grep -c "move" ~/reip/logs/robot_1_*.jsonl 2>&1')
print(f"Move entries: {stdout.read().decode().strip()}")

ssh.close()

# Also test: can we send UDP from this PC to the robot?
import socket
import json
import time
import os

# SSH password for the robot Pis. Set REIP_ROBOT_SSH_PASSWORD in your environment.
SSH_PASSWORD = os.environ.get("REIP_ROBOT_SSH_PASSWORD")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

msg = json.dumps({
    'type': 'position',
    'robot_id': 1,
    'x': 500.0,
    'y': 400.0,
    'theta': 0.0,
    'timestamp': time.time()
}).encode()

print("\n=== Sending test UDP to 255.255.255.255:5100 ===")
try:
    sock.sendto(msg, ('255.255.255.255', 5100))
    print("Sent via broadcast")
except Exception as e:
    print(f"Broadcast failed: {e}")

print("\n=== Sending test UDP to 192.168.20.16:5100 (clanker1 IP) ===")
try:
    sock.sendto(msg, ('192.168.20.16', 5100))
    print("Sent via unicast")
except Exception as e:
    print(f"Unicast failed: {e}")

sock.close()
