import paramiko, time
import os

# SSH password for the robot Pis. Set REIP_ROBOT_SSH_PASSWORD in your environment.
SSH_PASSWORD = os.environ.get("REIP_ROBOT_SSH_PASSWORD")
hosts = {1:'192.168.20.16', 2:'192.168.20.15', 3:'192.168.20.112', 4:'192.168.20.22', 5:'192.168.20.18'}
for r, h in hosts.items():
    try:
        s = paramiko.SSHClient()
        s.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        s.connect(h, username='pi', password=SSH_PASSWORD, timeout=5)
        s.exec_command('pkill -f reip_node; pkill -f raft_node; sleep 0.5; pkill -9 -f python3')
        s.close()
        print(f'R{r}: killed')
    except Exception as e:
        print(f'R{r}: FAILED ({e})')
