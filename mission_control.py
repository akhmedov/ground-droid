import cv2
import time
import argparse
import numpy as np
from zmq_motor import ZmqMotor
from gst_camera import ZmqCamera

SPEED = 0.1
EDGE_STATION_IP = '192.168.31.101'
LEFT_CAMERA_PORT = 1807
RIGHT_CAMERA_PORT = 1808
MOTOR_CTL_PORT = 3434


def print_user_manual():
    print('w: forward')
    print('s: backward')
    print('a: turn left')
    print('d: turn right')
    print('0-9: speed choice')
    print('q: quit program')


def image_preprocessing(frame: np.array):
    frame = cv2.flip(frame, -1)
    dim = int(1.2 * 2 * frame.shape[1]), int(2 * frame.shape[0])
    return cv2.resize(frame, dim)


parser = argparse.ArgumentParser()
parser.add_argument('--vision', choices=['off', 'left', 'right', 'stereo'], default='left', help='vision camera mode')
args = parser.parse_args()

left_camera, right_camera, image = None, None, None
motor_driver = ZmqMotor(server_ip=EDGE_STATION_IP, server_port=MOTOR_CTL_PORT)
if args.vision == 'left' or args.vision == 'stereo':
    left_camera = ZmqCamera(ip=EDGE_STATION_IP, port=LEFT_CAMERA_PORT)
if args.vision == 'right' or args.vision == 'stereo':
    right_camera = ZmqCamera(ip=EDGE_STATION_IP, port=RIGHT_CAMERA_PORT)
time.sleep(1)
print_user_manual()

while True:

    if left_camera and right_camera:
        stacked = np.hstack((left_camera.value, right_camera.value))
        image = image_preprocessing(stacked)
        cv2.imshow('Stereo camera image', image)
    elif left_camera:
        image = image_preprocessing(left_camera.value)
        cv2.imshow('Left camera image', image)
    elif right_camera:
        image = image_preprocessing(right_camera.value)
        cv2.imshow('Right camera image', image)
    else:
        dumpy = np.zeros((300, 300, 3))
        dumpy = cv2.putText(dumpy, 'No image', (80, 150),
                            cv2.FONT_HERSHEY_SIMPLEX, 1,
                            (0, 0, 255), 2, cv2.LINE_AA)
        cv2.imshow('No camera image', dumpy)

    key = cv2.waitKey(1)
    if ord('0') <= key <= ord('9'):
        SPEED = (key + 1 - 48) / 10
    elif key == ord('w'):
        motor_driver.forward(SPEED)
    elif key == ord('s'):
        motor_driver.backward(SPEED)
    elif key == ord('a'):
        motor_driver.left(SPEED)
    elif key == ord('d'):
        motor_driver.right(SPEED)
    elif key == ord('q'):
        if left_camera:
            left_camera.stop()
        if right_camera:
            right_camera.stop()
        cv2.destroyAllWindows()
        exit(1)
