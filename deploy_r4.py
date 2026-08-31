import paramiko, time, os
import os

# SSH password for the robot Pis. Set REIP_ROBOT_SSH_PASSWORD in your environment.
SSH_PASSWORD = os.environ.get("REIP_ROBOT_SSH_PASSWORD")

LOCAL_FILE = os.path.join(os.path.dirname(__file__), 'robot', 'reip_node.py')

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
print("Connecting to clanker4 (192.168.20.22)...")
ssh.connect('192.168.20.22', username='pi', password=SSH_PASSWORD, timeout=10)

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

ssh.exec_command('nohup python3 /home/pi/reip/reip_node.py 4 > /tmp/reip_4.log 2>&1 &')
time.sleep(1.5)

stdin, stdout, stderr = ssh.exec_command('pgrep -f reip_node')
pid = stdout.read().decode().strip()
print(f"  pid={pid}" if pid else "  FAILED TO START")
ssh.close()
