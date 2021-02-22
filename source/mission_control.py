import cv2
import time
import argparse
import numpy as np
from zmq_motor import ZmqMotor
from gst_camera import ZmqCamera
from calibration_store import load_stereo_coefficients

SPEED = 0.1
EDGE_STATION_IP = '192.168.31.101' # '192.168.2.3'
LEFT_CAMERA_PORT = 1807
RIGHT_CAMERA_PORT = 1808
MOTOR_CTL_PORT = 3434
WIDTH = 400
HEIGHT = 300
EXTRINSIC = 'calibration_data/extrinsic.xml'
K1, D1, K2, D2, R, T, E, F, R1, R2, P1, P2, Q = load_stereo_coefficients(EXTRINSIC)
IMAGE_SEPARATOR = np.zeros(shape=(HEIGHT, 10, 3), dtype=np.uint8)
IMAGE_SEPARATOR[0::5, :] = 255, 255, 255
WINDOW_SIZE = 3


def depth_map_left(left_frame, right_frame, lmbda=80000, sigma=1.3):
    left_map_x, left_map_y = cv2.initUndistortRectifyMap(K1, D1, R1, P1, (WIDTH, HEIGHT), cv2.CV_32FC1)
    left_rectified = cv2.remap(left_frame, left_map_x, left_map_y, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)
    right_map_x, right_map_y = cv2.initUndistortRectifyMap(K2, D2, R2, P2, (WIDTH, HEIGHT), cv2.CV_32FC1)
    right_rectified = cv2.remap(right_frame, right_map_x, right_map_y, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)
    gray_rectified_left = cv2.cvtColor(left_rectified, cv2.COLOR_BGR2GRAY)
    gray_rectified_right = cv2.cvtColor(right_rectified, cv2.COLOR_BGR2GRAY)
    wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
    wls_filter.setLambda(lmbda)
    wls_filter.setSigmaColor(sigma)
    displ = left_matcher.compute(gray_rectified_left, gray_rectified_right)  # .astype(np.float32)/16
    dispr = right_matcher.compute(gray_rectified_right, gray_rectified_left)  # .astype(np.float32)/16
    displ = np.int16(displ)
    dispr = np.int16(dispr)
    filteredImg = wls_filter.filter(displ, gray_rectified_left, None, dispr)  # important to put "imgL" here!!!
    filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX)
    return np.uint8(filteredImg)


def print_user_manual():
    print('w: forward')
    print('s: backward')
    print('a: turn left')
    print('d: turn right')
    print('0-9: speed choice')
    print('q: quit program')


def image_preproc(img, scale=1.5):
    width = int(img.shape[1] * scale)
    height = int(img.shape[0] * scale)
    return cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)


parser = argparse.ArgumentParser()
parser.add_argument('--vision', choices=['off', 'left', 'right', 'stereo', 'depth'], default='left', help='vision camera mode')
args = parser.parse_args()

left_camera, right_camera, image, left_matcher, right_matcher = None, None, None, None, None
motor_driver = ZmqMotor(server_ip=EDGE_STATION_IP, server_port=MOTOR_CTL_PORT)
if args.vision == 'left' or args.vision == 'stereo' or args.vision == 'depth':
    left_camera = ZmqCamera(ip=EDGE_STATION_IP, port=LEFT_CAMERA_PORT, width=WIDTH, height=HEIGHT)
if args.vision == 'right' or args.vision == 'stereo' or args.vision == 'depth':
    right_camera = ZmqCamera(ip=EDGE_STATION_IP, port=RIGHT_CAMERA_PORT, width=WIDTH, height=HEIGHT)
if args.vision == 'depth':
    left_matcher = cv2.StereoSGBM_create(
        minDisparity=-1,
        numDisparities=5 * 16,  # max_disp has to be dividable by 16 f. E. HH 192, 256
        blockSize=WINDOW_SIZE,
        P1=8*3*WINDOW_SIZE,  # wsize default 3; 5; 7 for SGBM reduced size image;
        P2=32*3*WINDOW_SIZE,
        disp12MaxDiff=12,
        uniquenessRatio=10,
        speckleWindowSize=50,
        speckleRange=32,
        preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )
    right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)

time.sleep(1)
print_user_manual()

while True:

    if left_camera and right_camera and left_matcher and right_matcher:
        depth = depth_map_left(left_camera.value, right_camera.value)
        leftMapX, leftMapY = cv2.initUndistortRectifyMap(K1, D1, R1, P1, (WIDTH, HEIGHT), cv2.CV_32FC1)
        left_rectified = cv2.remap(left_camera.value, leftMapX, leftMapY, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)
        depth = cv2.cvtColor(depth[..., np.newaxis], cv2.COLOR_GRAY2RGB)
        stacked = np.hstack((left_rectified, IMAGE_SEPARATOR, depth))
        stacked = image_preproc(stacked)
        cv2.imshow('Stereo camera image', stacked)
    elif left_camera and right_camera:
        stacked = np.hstack((left_camera.value, IMAGE_SEPARATOR, right_camera.value))
        stacked = image_preproc(stacked)
        cv2.imshow('Stereo camera image', stacked)
    elif left_camera:
        image = image_preproc(left_camera.value)
        cv2.imshow('Left camera image', image)
    elif right_camera:
        image = image_preproc(right_camera.value)
        cv2.imshow('Right camera image', image)
    else:
        dumpy = np.zeros((300, 400, 3))
        dumpy = cv2.putText(dumpy, 'No image', (130, 150),
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