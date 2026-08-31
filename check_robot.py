import paramiko
import os

# SSH password for the robot Pis. Set REIP_ROBOT_SSH_PASSWORD in your environment.
SSH_PASSWORD = os.environ.get("REIP_ROBOT_SSH_PASSWORD")
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('clanker1.local', username='pi', password=SSH_PASSWORD, timeout=10)

print("=== Log files ===")
stdin, stdout, stderr = ssh.exec_command('ls -la ~/reip/logs/ 2>&1')
print(stdout.read().decode())

print("=== /tmp/reip_1.log ===")
stdin, stdout, stderr = ssh.exec_command('cat /tmp/reip_1.log 2>&1')
out = stdout.read().decode()
print(out[-2000:] if len(out) > 2000 else out if out else "(empty)")

print("=== Check UART ===")
stdin, stdout, stderr = ssh.exec_command('ls -la /dev/serial* /dev/ttyS* /dev/ttyAMA* 2>&1')
print(stdout.read().decode())

print("=== UDP port 5100 test (2s) ===")
stdin, stdout, stderr = ssh.exec_command(
    'timeout 2 python3 -u -c "'
    'import socket; s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);'
    's.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);'
    's.bind((chr(0)*0,5100));'
    's.settimeout(2);'
    'print(s.recvfrom(1024))" 2>&1'
)
print(stdout.read().decode())

ssh.close()
