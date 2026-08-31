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
        stdin, stdout, stderr = ssh.exec_command(f'tail -3 /tmp/reip_{rid}.log')
        lines = stdout.read().decode().strip()
        if lines:
            for l in lines.split('\n')[-2:]:
                print(f"R{rid}: {l}")
        else:
            print(f"R{rid}: (still booting)")
        ssh.close()
    except Exception as e:
        print(f"R{rid}: {e}")
