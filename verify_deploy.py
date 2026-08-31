import paramiko
import os

# SSH password for the robot Pis. Set REIP_ROBOT_SSH_PASSWORD in your environment.
SSH_PASSWORD = os.environ.get("REIP_ROBOT_SSH_PASSWORD")

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('192.168.20.16', username='pi', password=SSH_PASSWORD, timeout=5)

# Check if code has _wall_slide_heading (from latest deploy)
stdin, stdout, stderr = ssh.exec_command(
    'grep -c "_wall_slide_heading" /home/pi/reip/reip_node.py')
print(f"_wall_slide_heading occurrences: {stdout.read().decode().strip()}")

# Check if NEW JSONL exists
stdin, stdout, stderr = ssh.exec_command(
    'ls -lt ~/reip/logs/robot_1_*.jsonl | head -3')
print(f"Log files:\n{stdout.read().decode().strip()}")

# Check first 5 lines of /tmp/reip_1.log (startup messages)
stdin, stdout, stderr = ssh.exec_command('head -20 /tmp/reip_1.log')
print(f"\nStartup log:\n{stdout.read().decode().strip()}")

# Check last line for current state 
stdin, stdout, stderr = ssh.exec_command('tail -2 /tmp/reip_1.log')
print(f"\nCurrent state:\n{stdout.read().decode().strip()}")

# Is position_timestamp > 0?  Check by counting cells visited
stdin, stdout, stderr = ssh.exec_command(
    'grep -c "position" /tmp/reip_1.log')
print(f"\nPosition mentions: {stdout.read().decode().strip()}")

ssh.close()
