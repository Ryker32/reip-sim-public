import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('192.168.20.15', username='pi', password=SSH_PASSWORD, timeout=5)

# Full startup log
stdin, stdout, stderr = ssh.exec_command('head -30 /tmp/reip_2.log')
print("Startup:")
print(stdout.read().decode())

# Also check what ToF reads are
stdin, stdout, stderr = ssh.exec_command('''python3 -c "
import smbus, busio, board, adafruit_vl53l0x, time
import os

# SSH password for the robot Pis. Set REIP_ROBOT_SSH_PASSWORD in your environment.
SSH_PASSWORD = os.environ.get("REIP_ROBOT_SSH_PASSWORD")
bus = smbus.SMBus(1)
i2c = busio.I2C(board.SCL, board.SDA)
MUX = 0x70
channels = {0:'right', 1:'front_right', 2:'front', 3:'front_left', 4:'left'}
for ch, name in channels.items():
    try:
        bus.write_byte(MUX, 1 << ch)
        time.sleep(0.05)
        s = adafruit_vl53l0x.VL53L0X(i2c)
        readings = [s.range for _ in range(5)]
        print(f'  {name}: {readings}')
    except Exception as e:
        print(f'  {name}: FAILED ({e})')
"''')
print("ToF readings:")
print(stdout.read().decode())
print(stderr.read().decode())

ssh.close()
