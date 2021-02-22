from flask import send_file, Flask, request, Response, json
from gst_camera import ZmqCamera
from zmq_motor import ZmqMotor
import numpy as np
import cv2
import time
import sys

# Initialize the Flask application
app = Flask(__name__)


@app.route('/left_photo', methods=['GET'])
def left_photo():
    camera = ZmqCamera(ip='127.0.0.1', port=1807, height=300, width=400)
    time.sleep(2)
    cv2.imwrite('/tmp/last_photo.png', camera.value)
    camera.stop()
    return send_file('/tmp/last_photo.png', mimetype='image/png')


@app.route('/right_photo', methods=['GET'])
def right_photo():
    camera = ZmqCamera(ip='127.0.0.1', port=1808, height=300, width=400)
    time.sleep(2)
    cv2.imwrite('/tmp/last_photo.png', camera.value)
    camera.stop()
    return send_file('/tmp/last_photo.png', mimetype='image/png')


@app.route('/sensor_data', methods=['GET'])
def sensor_data():
    from icm20948 import ICM20948
    imu = ICM20948()
    x, y, z = imu.read_magnetometer_data()
    ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()
    heading_azimuth = 90 - np.arctan2(x, y) * 180 / np.pi
    data = {'heading_azimuth': heading_azimuth}
    response = app.response_class(
        response=json.dumps(data),
        status=200,
        mimetype='application/json'
    )
    return response




@app.route('/set_protocol', methods=['GET'])
def get_flatter():
    name = request.args.get('name', default='flatter', type=str)
    if name == 'flatter':
        motor_driver = ZmqMotor(server_ip='127.0.0.1', server_port=3434)
        start = time.time()
        while time.time() - start < 2:
            motor_driver.left(0.2)
            time.sleep(0.07)
            motor_driver.right(0.2)
            time.sleep(0.07)
        motor_driver.stop()
    response = app.response_class(
            response=json.dumps({"status": "success", "code": 0}),
            status=200,
            mimetype='application/json'
    )
    return response


# start flask app
app.run(host='0.0.0.0', port=5000)
