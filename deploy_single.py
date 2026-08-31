import paramiko, time, os, sys
import os

# SSH password for the robot Pis. Set REIP_ROBOT_SSH_PASSWORD in your environment.
SSH_PASSWORD = os.environ.get("REIP_ROBOT_SSH_PASSWORD")

HOSTS = {1:'192.168.20.16',2:'192.168.20.15',3:'192.168.20.112',4:'192.168.20.22',5:'192.168.20.18'}
rid = int(sys.argv[1]) if len(sys.argv) > 1 else 5
host = HOSTS[rid]
LOCAL_FILE = os.path.join(os.path.dirname(__file__), 'robot', 'reip_node.py')

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
print(f"Connecting to R{rid} ({host})...")
ssh.connect(host, username='pi', password=SSH_PASSWORD, timeout=10)
ssh.exec_command('pkill -f reip_node; sleep 0.5')
time.sleep(1)
sftp = ssh.open_sftp()
try: sftp.mkdir('/home/pi/reip')
except: pass
try: sftp.mkdir('/home/pi/reip/logs')
except: pass
sftp.put(LOCAL_FILE, '/home/pi/reip/reip_node.py')
sftp.close()
print("  deployed")
ssh.exec_command(f'nohup python3 /home/pi/reip/reip_node.py {rid} > /tmp/reip_{rid}.log 2>&1 &')
time.sleep(1.5)
stdin, stdout, stderr = ssh.exec_command('pgrep -f reip_node')
pid = stdout.read().decode().strip()
print(f"  R{rid}: {'RUNNING pid=' + pid if pid else 'FAILED'}")
ssh.close()
