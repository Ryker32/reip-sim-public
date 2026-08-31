import paramiko, json, time
import os

# SSH password for the robot Pis. Set REIP_ROBOT_SSH_PASSWORD in your environment.
SSH_PASSWORD = os.environ.get("REIP_ROBOT_SSH_PASSWORD")

HOSTS = {
    1: '192.168.20.16',
    2: '192.168.20.15',
    3: '192.168.20.112',
    4: '192.168.20.22',
    5: '192.168.20.18',
}

for rid in sorted(HOSTS.keys()):
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(HOSTS[rid], username='pi', password=SSH_PASSWORD, timeout=5)

        # Is process running?
        stdin, stdout, stderr = ssh.exec_command('pgrep -f reip_node')
        pid = stdout.read().decode().strip()

        # Last 3 lines of stderr log (status prints)
        stdin, stdout, stderr = ssh.exec_command(f'tail -3 /tmp/reip_{rid}.log')
        log = stdout.read().decode().strip()

        # Last JSONL entry (actual data)
        stdin, stdout, stderr = ssh.exec_command(
            f'ls -t ~/reip/logs/robot_{rid}_*.jsonl 2>/dev/null | head -1')
        logfile = stdout.read().decode().strip()

        nav_target = None
        pos_ts = None
        state = None
        x = y = 0
        visited = 0
        if logfile:
            stdin, stdout, stderr = ssh.exec_command(f'tail -1 {logfile}')
            line = stdout.read().decode().strip()
            if line:
                d = json.loads(line)
                x, y = d.get('x', 0), d.get('y', 0)
                state = d.get('state')
                nav_target = d.get('navigation_target')
                visited = d.get('known_visited_count', 0)
                pos_ts = d.get('timestamp', 0)

        print(f"R{rid}: pid={'ALIVE' if pid else 'DEAD':5s} "
              f"pos=({x:.0f},{y:.0f}) state={state} "
              f"nav_target={nav_target} visited={visited}")
        print(f"      log: {log.split(chr(10))[-1] if log else 'EMPTY'}")
        ssh.close()
    except Exception as e:
        print(f"R{rid}: ERROR - {e}")
