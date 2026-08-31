import paramiko
import os

# SSH password for the robot Pis. Set REIP_ROBOT_SSH_PASSWORD in your environment.
SSH_PASSWORD = os.environ.get("REIP_ROBOT_SSH_PASSWORD")

hosts = {1:'192.168.20.16',2:'192.168.20.15',3:'192.168.20.112'}

for rid in [1, 2]:
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(hosts[rid], username='pi', password=SSH_PASSWORD, timeout=5)
        
        stdin, stdout, stderr = ssh.exec_command(f'tail -30 /tmp/reip_{rid}.log')
        print(f"=== clanker{rid} stderr ===")
        print(stdout.read().decode())
        ssh.close()
    except Exception as e:
        print(f"R{rid}: {e}")
