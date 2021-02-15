# import os
# os.spawnl(os.P_DETACH, 'some_long_running_command')

# import subprocess
# cmd = ['/usr/bin/python', '/path/to/my/second/pythonscript.py']
# subprocess.Popen(cmd, stdout=sys.stdout, stderr=sys.stderr).wait()

# pwm motor interface initialization
sudo busybox devmem 0x700031fc 32 0x45
sudo busybox devmem 0x6000d504 32 0x2
sudo busybox devmem 0x70003248 32 0x46
sudo busybox devmem 0x6000d100 32 0x00

# zmq camera pablishers for stereo camera
python3 zmq_camera_publisher.py --sensor_id 0 --port 1807 &
python3 zmq_camera_publisher.py --sensor_id 1 --port 1808 &

# zmq motor pablishers for two pwm motor arch
python3 zmq_motor_publisher.py --port 3434 &
