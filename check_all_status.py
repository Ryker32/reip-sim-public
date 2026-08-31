import paramiko
import os

# SSH password for the robot Pis. Set REIP_ROBOT_SSH_PASSWORD in your environment.
SSH_PASSWORD = os.environ.get("REIP_ROBOT_SSH_PASSWORD")

HOSTS = {1:'192.168.20.16', 2:'192.168.20.15', 3:'192.168.20.112', 4:'192.168.20.22', 5:'192.168.20.18'}

for rid in sorted(HOSTS.keys()):
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(HOSTS[rid], username='pi', password=SSH_PASSWORD, timeout=5)
        
        # Check if process is running
        stdin, stdout, stderr = ssh.exec_command(f'pgrep -f "reip_node.py"')
        pids = stdout.read().decode().strip()
        
        # Check log size
        stdin, stdout, stderr = ssh.exec_command(f'wc -l /tmp/reip_{rid}.log 2>/dev/null')
        log_lines = stdout.read().decode().strip()
        
        # Last 3 log lines
        stdin, stdout, stderr = ssh.exec_command(f'tail -3 /tmp/reip_{rid}.log 2>/dev/null')
        tail = stdout.read().decode().strip()
        
        print(f"R{rid}: pids=[{pids}] log_lines={log_lines}")
        if tail:
            for l in tail.split('\n'):
                print(f"     {l}")
        print()
        ssh.close()
    except Exception as e:
        print(f"R{rid}: {e}\n")
