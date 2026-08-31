import paramiko, time

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('192.168.20.22', username='pi', password=SSH_PASSWORD, timeout=5)

# Kill any running reip_node first so we can use UART
ssh.exec_command('pkill -f reip_node; sleep 1')
time.sleep(2)

# Direct UART motor test
test_script = '''
import serial
import time
import os

# SSH password for the robot Pis. Set REIP_ROBOT_SSH_PASSWORD in your environment.
SSH_PASSWORD = os.environ.get("REIP_ROBOT_SSH_PASSWORD")

uart = serial.Serial('/dev/serial0', 115200, timeout=0.5)
time.sleep(0.3)
uart.reset_input_buffer()

# Ping pico
uart.write(b'PING\\n')
time.sleep(0.1)
resp = uart.readline().decode().strip()
print(f'PING response: {resp}')

# Run motors for 2 seconds
print('Setting motors to 70,70...')
uart.write(b'MOT,70.0,70.0\\n')
time.sleep(0.1)
resp = uart.readline().decode().strip()
print(f'MOT response: {resp}')

time.sleep(2)

# Stop
uart.write(b'STOP\\n')
time.sleep(0.1)
resp = uart.readline().decode().strip()
print(f'STOP response: {resp}')
print('Done - did wheels spin?')

uart.close()
'''

stdin, stdout, stderr = ssh.exec_command(f'python3 -c "{test_script}"')
print("STDOUT:", stdout.read().decode())
print("STDERR:", stderr.read().decode())
ssh.close()
