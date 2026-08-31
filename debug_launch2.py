import paramiko, time
import os

# SSH password for the robot Pis. Set REIP_ROBOT_SSH_PASSWORD in your environment.
SSH_PASSWORD = os.environ.get("REIP_ROBOT_SSH_PASSWORD")

HOSTS = {1:'192.168.20.16', 3:'192.168.20.112', 4:'192.168.20.22'}

for rid in sorted(HOSTS.keys()):
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(HOSTS[rid], username='pi', password=SSH_PASSWORD, timeout=10)

        # Check CORRECT file location
        stdin, stdout, stderr = ssh.exec_command('ls -la /home/pi/reip/reip_node.py')
        print(f"R{rid} file: {stdout.read().decode().strip()}")

        # Check log
        stdin, stdout, stderr = ssh.exec_command(f'cat /tmp/reip_{rid}.log 2>/dev/null | head -30')
        log = stdout.read().decode().strip()
        print(f"R{rid} log:\n{log if log else '(empty)'}")
        
        # Check stderr separately 
        stdin, stdout, stderr = ssh.exec_command(f'cat /tmp/reip_{rid}.log 2>/dev/null | tail -10')
        tail = stdout.read().decode().strip()
        if tail and tail != log:
            print(f"R{rid} log tail:\n{tail}")
        
        # Try direct launch and capture error
        print(f"\nR{rid} direct test:")
        stdin, stdout, stderr = ssh.exec_command(
            f'cd /home/pi/reip && timeout 8 python3 reip_node.py {rid} 2>&1 | head -15'
        )
        out = stdout.read().decode().strip()
        print(f"  {out}")
        print()
        ssh.close()
    except Exception as e:
        print(f"R{rid}: {e}\n")
