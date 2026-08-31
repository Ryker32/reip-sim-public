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

        # Check if file exists
        stdin, stdout, stderr = ssh.exec_command('ls -la /home/pi/reip_node.py')
        print(f"R{rid} file: {stdout.read().decode().strip()}")
        err = stderr.read().decode().strip()
        if err:
            print(f"  err: {err}")

        # Try running with direct stderr capture
        stdin, stdout, stderr = ssh.exec_command(
            f'timeout 10 python3 /home/pi/reip_node.py --robot-id {rid} 2>&1 | head -20'
        )
        out = stdout.read().decode().strip()
        print(f"R{rid} output:\n{out}")
        print()
        ssh.close()
    except Exception as e:
        print(f"R{rid}: {e}\n")
