import paramiko, json
import os

# SSH password for the robot Pis. Set REIP_ROBOT_SSH_PASSWORD in your environment.
SSH_PASSWORD = os.environ.get("REIP_ROBOT_SSH_PASSWORD")

HOSTS = {1:'192.168.20.16', 2:'192.168.20.15', 3:'192.168.20.112', 4:'192.168.20.22', 5:'192.168.20.18'}

for rid in [1, 3]:
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(HOSTS[rid], username='pi', password=SSH_PASSWORD, timeout=5)

        # List ALL JSONL files
        stdin, stdout, stderr = ssh.exec_command(
            f'ls -lt ~/reip/logs/robot_{rid}_*.jsonl 2>/dev/null')
        print(f"=== R{rid} log files ===")
        print(stdout.read().decode().strip())

        # Get the NEWEST file
        stdin, stdout, stderr = ssh.exec_command(
            f'ls -t ~/reip/logs/robot_{rid}_*.jsonl 2>/dev/null | head -1')
        newest = stdout.read().decode().strip()
        
        if newest:
            # Line count
            stdin, stdout, stderr = ssh.exec_command(f'wc -l {newest}')
            count = stdout.read().decode().strip()
            print(f"  newest: {newest} ({count})")
            
            # Last 3 entries
            stdin, stdout, stderr = ssh.exec_command(f'tail -3 {newest}')
            for line in stdout.read().decode().strip().split('\n'):
                if line.strip():
                    d = json.loads(line)
                    print(f"  pos=({d.get('x',0):.0f},{d.get('y',0):.0f}) "
                          f"nav={d.get('navigation_target')} "
                          f"pred={d.get('predicted_target')} "
                          f"leader_self={d.get('leader_self_target')} "
                          f"my_vis={d.get('my_visited_count')} "
                          f"known_vis={d.get('known_visited_count')}")
        print()
        ssh.close()
    except Exception as e:
        print(f"R{rid}: {e}")
