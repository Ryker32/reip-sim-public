import paramiko, json, time
import os

# SSH password for the robot Pis. Set REIP_ROBOT_SSH_PASSWORD in your environment.
SSH_PASSWORD = os.environ.get("REIP_ROBOT_SSH_PASSWORD")

hosts = {1:'192.168.20.16',2:'192.168.20.15',3:'192.168.20.112',4:'192.168.20.22',5:'192.168.20.18'}

for rid in [1, 2, 3]:
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(hosts[rid], username='pi', password=SSH_PASSWORD, timeout=5)
        
        # Is process running?
        stdin, stdout, stderr = ssh.exec_command('pgrep -f reip_node')
        pid = stdout.read().decode().strip()
        
        # Last log line
        stdin, stdout, stderr = ssh.exec_command(f'ls -t ~/reip/logs/robot_{rid}_*.jsonl 2>/dev/null | head -1')
        logfile = stdout.read().decode().strip()
        
        if logfile:
            stdin, stdout, stderr = ssh.exec_command(f'tail -1 {logfile}')
            line = stdout.read().decode().strip()
            if line:
                d = json.loads(line)
                print(f"R{rid}: pid={pid} pos=({d['x']:.0f},{d['y']:.0f}) state={d['state']} target={d.get('commanded_target')} visited={d['known_visited_count']}")
            else:
                print(f"R{rid}: pid={pid} log empty")
        else:
            print(f"R{rid}: pid={pid} no log file")
        ssh.close()
    except Exception as e:
        print(f"R{rid}: {e}")
