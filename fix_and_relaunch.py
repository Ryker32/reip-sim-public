import paramiko, time
import os

# SSH password for the robot Pis. Set REIP_ROBOT_SSH_PASSWORD in your environment.
SSH_PASSWORD = os.environ.get("REIP_ROBOT_SSH_PASSWORD")

HOSTS = {1:'192.168.20.16', 2:'192.168.20.15', 3:'192.168.20.112', 4:'192.168.20.22', 5:'192.168.20.18'}

for rid in [3, 4]:
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(HOSTS[rid], username='pi', password=SSH_PASSWORD, timeout=5)

        # Check crash log
        stdin, stdout, stderr = ssh.exec_command(f'tail -15 /tmp/reip_{rid}.log 2>/dev/null')
        log = stdout.read().decode().strip()
        print(f"=== R{rid} crash log ===")
        print(log if log else "(empty)")
        print()

        # Relaunch
        ssh.exec_command('pkill -f reip_node.py')
        time.sleep(1)
        cmd = f'nohup python3 /home/pi/reip_node.py --robot-id {rid} > /tmp/reip_{rid}.log 2>&1 &'
        ssh.exec_command(cmd)
        time.sleep(2)

        # Verify
        stdin, stdout, stderr = ssh.exec_command('pgrep -f reip_node.py')
        pid = stdout.read().decode().strip()
        print(f"  R{rid}: {'RUNNING pid=' + pid if pid else 'FAILED TO LAUNCH'}")

        ssh.close()
    except Exception as e:
        print(f"R{rid}: {e}")

# Also try R1
print("\n=== R1 connectivity test ===")
try:
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(HOSTS[1], username='pi', password=SSH_PASSWORD, timeout=10)
    stdin, stdout, stderr = ssh.exec_command('hostname')
    print(f"  R1 hostname: {stdout.read().decode().strip()}")
    
    # Relaunch R1 too
    ssh.exec_command('pkill -f reip_node.py')
    time.sleep(1)
    cmd = 'nohup python3 /home/pi/reip_node.py --robot-id 1 > /tmp/reip_1.log 2>&1 &'
    ssh.exec_command(cmd)
    time.sleep(2)
    stdin, stdout, stderr = ssh.exec_command('pgrep -f reip_node.py')
    pid = stdout.read().decode().strip()
    print(f"  R1: {'RUNNING pid=' + pid if pid else 'FAILED TO LAUNCH'}")
    ssh.close()
except Exception as e:
    print(f"  R1: {e}")
