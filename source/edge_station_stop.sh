ps -ef | grep 'zmq_camera_publisher.py' | grep -v grep | awk '{print $2}' | xargs -r kill -9
ps -ef | grep 'zmq_camera_publisher.py' | grep -v grep | awk '{print $2}' | xargs -r kill -9
ps -ef | grep 'zmq_motor_publisher.py' | grep -v grep | awk '{print $2}' | xargs -r kill -9
ps -ef | grep 'sensor_server.py' | grep -v grep | awk '{print $2}' | xargs -r kill -9
