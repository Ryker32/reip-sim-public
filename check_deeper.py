import paramiko
import os

# SSH password for the robot Pis. Set REIP_ROBOT_SSH_PASSWORD in your environment.
SSH_PASSWORD = os.environ.get("REIP_ROBOT_SSH_PASSWORD")

hosts = {
    1: '192.168.20.16',
    2: '192.168.20.15',
    3: '192.168.20.112',
    4: '192.168.20.22',
    5: '192.168.20.18'
}

for rid in sorted(hosts.keys()):
    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(hosts[rid], username='pi', password=SSH_PASSWORD, timeout=5)

        # Check ALL python processes
        stdin, stdout, stderr = ssh.exec_command('ps aux | grep python')
        ps = stdout.read().decode().strip()

        # Check if reip_node specifically is running
        stdin, stdout, stderr = ssh.exec_command('pgrep -af reip')
        pgrep = stdout.read().decode().strip()

        # Check last 5 lines of stderr log
        stdin, stdout, stderr = ssh.exec_command(f'tail -5 /tmp/reip_{rid}.log 2>/dev/null')
        log = stdout.read().decode().strip()

        # Check dmesg for OOM or crashes
        stdin, stdout, stderr = ssh.exec_command('dmesg | tail -5')
        dmesg = stdout.read().decode().strip()

        print(f"=== R{rid} ({hosts[rid]}) ===")
        print(f"pgrep: {pgrep if pgrep else 'NONE'}")
        print(f"ps python: {ps if ps else 'NONE'}")
        print(f"log: {log}")
        print(f"dmesg: {dmesg}")
        print()
        ssh.close()
    except Exception as e:
        print(f"R{rid}: UNREACHABLE - {e}\n")
