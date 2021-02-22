import cv2
import time
import argparse
import numpy as np
import os
import glob
import sys
import re
from gst_camera import ZmqCamera
from calibration_store import load_coefficients, save_coefficients, save_stereo_coefficients, load_stereo_coefficients


IMG_WIDTH = 400
IMG_HEIGHT = 300
CHESSBOARD_WIDTH = 7
CHESSBOARD_HEIGHT = 7
CALIB_IMG_DIR = 'calibration_frames_good/'
LEFT_INTRINSIC = 'calibration_data/left_input.xml'
RIGHT_INTRINSIC = 'calibration_data/right_input.xml'
EXTRINSIC = 'calibration_data/extrinsic.xml'


def write_frames(dir_name, height, width, edge_server_ip='127.0.0.1', left_cam_port=1807, right_cam_port=1808, duration=60):
    left_camera = ZmqCamera(ip=edge_server_ip, port=left_cam_port, height=height, width=width)
    right_camera = ZmqCamera(ip=edge_server_ip, port=right_cam_port, height=height, width=width)
    start_moment = time.time()
    frame_number = 0
    try:
        while time.time() - start_moment < duration:
            frame_number += 1
            left_filename = dir_name + '/left_' + str(frame_number).zfill(5) + '.png'
            right_filename = dir_name + '/right_' + str(frame_number).zfill(5) + '.png'
            left_image = left_camera.value
            right_image = right_camera.value
            cv2.imwrite(left_filename, left_image)
            cv2.imwrite(right_filename, right_image)
    except KeyboardInterrupt:
        print("Press Ctrl-C to terminate while statement")
        left_camera.stop()
        right_camera.stop()
    return frame_number


def validate_calibration_images(dir_name):
    images = glob.glob(dir_name + '/*.png')
    left = list()
    right = list()
    for fname in images:
        if 'left_' in fname:
            left.append(fname)
        elif 'right_' in fname:
            right.append(fname)
        else:
            print('Fatal error')
            sys.exit(0)
    if len(left) != len(right):
        print('length not equal')
    left.sort()
    right.sort()
    return left, right


def define_chessboard_corners_3d(items, chess_square_size=0.0309, chess_width=7, chess_height=7):
    """Real 3D coordinates of chessboard corners."""
    objp = np.zeros((chess_height * chess_width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:chess_width, 0:chess_height].T.reshape(-1, 2)
    objp = objp * chess_square_size
    return [objp] * items


def find_chessboard_corners_2d(images_path, window_size, find_flags=None, chess_width=7, chess_height=7):
    """Finds 2D coordinates of chessboard corners in image plane."""
    imgpoints = dict()
    criteria = cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001
    for path in images_path:
        image = cv2.imread(path)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (chess_width, chess_height), find_flags)
        if ret:
            corners2 = cv2.cornerSubPix(gray, corners, window_size, (-1, -1), criteria)
            idx = os.path.splitext(os.path.basename(path))[0]
            idx = int(re.search(r'\d+', idx).group())
            if imgpoints.get(idx, None):
                print('already exists', idx)
                raise IndexError("Wrong indexing - already exists")
            imgpoints[idx] = corners2
    return imgpoints


def intrinsic_calibration(images_path: list, img_width: int, img_height: int, used_image_ratio: int):
    """Find intrinsic calibration parameters for pinhol camera."""
    imgpoints_dict = find_chessboard_corners_2d(images_path, window_size=(11, 11), find_flags=None)
    imgpoints = list(imgpoints_dict.values())
    imgpoints = imgpoints[0::used_image_ratio]
    objpoints = define_chessboard_corners_3d(len(imgpoints))
    print('Starting intrinsic calibration for frames:', len(imgpoints))
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (img_width, img_height), None, None)
    return ret, mtx, dist, rvecs, tvecs


def extrinsic_calibration(left_path: list, right_path: list, img_width: int, img_height: int, used_image_ratio: int):
    """Find intrinsic and extrinsic calibration parameters for stereo pair of pinhol cameras."""
    leftp_dict = find_chessboard_corners_2d(left_path, window_size=(5, 5),
                                            find_flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FILTER_QUADS)
    rightp_dict = find_chessboard_corners_2d(right_path, window_size=(5, 5),
                                             find_flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FILTER_QUADS)
    good_couples_path = set(leftp_dict).intersection(rightp_dict)
    leftp = [leftp_dict.get(key) for key in good_couples_path]
    rightp = [rightp_dict.get(key) for key in good_couples_path]
    leftp = leftp[0::used_image_ratio]
    rightp = rightp[0::used_image_ratio]
    objp = define_chessboard_corners_3d(len(leftp))
    print('Begin stereo calibration for frames: ', len(objp))
    K1, D1 = load_coefficients(LEFT_INTRINSIC)
    K2, D2 = load_coefficients(RIGHT_INTRINSIC)
    ret, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(objp, leftp, rightp, K1, D1, K2, D2, (img_height, img_width))
    R1, R2, P1, P2, Q, roi_left, roi_right = cv2.stereoRectify(K1, D1, K2, D2, (img_height, img_width), R, T,
                                                               flags=cv2.CALIB_ZERO_DISPARITY, alpha=0.9)
    save_stereo_coefficients(EXTRINSIC, K1, D1, K2, D2, R, T, E, F, R1, R2, P1, P2, Q)


def draw_chess_board_pattern():
    start = time.time()
    left_frames, right_frames = validate_calibration_images(dir_name=CALIB_IMG_DIR)
    leftp_dict = find_chessboard_corners_2d(left_frames, window_size=(5, 5),
                                             find_flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FILTER_QUADS)
    rightp_dict = find_chessboard_corners_2d(right_frames, window_size=(5, 5),
                                             find_flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FILTER_QUADS)
    print('Time of pose estimation: ', time.time()-start)
    time.sleep(2)
    for path_left, path_right in zip(left_frames, right_frames):
        image_left = cv2.imread(path_left)
        image_right = cv2.imread(path_right)
        idx_left = os.path.splitext(os.path.basename(path_left))[0]
        idx_left = int(re.search(r'\d+', idx_left).group())
        if idx_left in leftp_dict:
            image_left = cv2.drawChessboardCorners(image_left, (CHESSBOARD_WIDTH, CHESSBOARD_HEIGHT), leftp_dict[idx_left], True)
        idx_right = os.path.splitext(os.path.basename(path_right))[0]
        idx_right = int(re.search(r'\d+', idx_right).group())
        if idx_right in rightp_dict:
            image_right = cv2.drawChessboardCorners(image_right, (CHESSBOARD_WIDTH, CHESSBOARD_HEIGHT), rightp_dict[idx_right], True)
        stacked = np.hstack((image_left, image_right))
        width = int(stacked.shape[1] * 2)
        height = int(stacked.shape[0] * 2)
        stacked = cv2.resize(stacked, (width, height), interpolation=cv2.INTER_AREA)
        cv2.imshow(str(idx_left) + ' ' + str(idx_right), stacked)
        time.sleep(2)
        cv2.waitKey(500)
    time.sleep(99999999)
    cv2.destroyAllWindows()


def calibrate_cameras():
    left_frames, right_frames = validate_calibration_images(dir_name=CALIB_IMG_DIR)
    print('Found images:', len(left_frames), len(right_frames))
    _, left_mtx, left_dist, _, _ = intrinsic_calibration(left_frames, img_width=IMG_WIDTH, img_height=IMG_HEIGHT, used_image_ratio=1)
    save_coefficients(left_mtx, left_dist, LEFT_INTRINSIC)
    print('Left camera intrinsic calibration done.')
    _, right_mtx, right_dist, _, _ = intrinsic_calibration(right_frames, img_width=IMG_WIDTH, img_height=IMG_HEIGHT, used_image_ratio=1)
    save_coefficients(right_mtx, right_dist, RIGHT_INTRINSIC)
    print('Right camera intrinsic calibration done.')
    extrinsic_calibration(left_frames, right_frames, img_width=IMG_WIDTH, img_height=IMG_HEIGHT, used_image_ratio=1)
    print('Stereo camera calibration done.')


# draw_chess_board_pattern()
calibrate_cameras()

# Write frames
# parser = argparse.ArgumentParser()
# parser.add_argument('--width', type=int)
# parser.add_argument('--height', type=int)
# parser.add_argument('--dir', type=str)
# parser.add_argument('--duration', type=int)
# parser.add_argument('--delay', type=int)
# args = parser.parse_args()
#
# time.sleep(args.delay)
# print('Record started...')
# frames = write_frames(args.dir, height=args.height, width=args.width, duration=args.duration)
# print('Wrote frames: ', frames)
