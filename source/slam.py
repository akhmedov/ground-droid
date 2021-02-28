import numpy as np
import cv2

from constants import LEFT_CAMERA_PORT, RIGHT_CAMERA_PORT, \
    IMG_WIDTH, IMG_HEIGHT, STREAMING_FPS, EXTRINSIC

WINDOW_SIZE = 3
IMG_SEPARATOR_WIDTH = 10


def init_image_separator(width=IMG_SEPARATOR_WIDTH, color=True):
    shape = (IMG_HEIGHT, width, 3) if color else (IMG_HEIGHT, width, 1)
    separator = np.zeros(shape=shape, dtype=np.uint8)
    separator[0::5, :] = 255, 255, 255
    return separator


def init_stereo_matchers(window_size=WINDOW_SIZE):
    left_matcher = cv2.StereoSGBM_create(
        minDisparity=-1,
        numDisparities=5 * 16,  # max_disp has to be dividable by 16 f. E. HH 192, 256
        blockSize=window_size,
        P1=8 * 3 * window_size,  # wsize default 3; 5; 7 for SGBM reduced size image;
        P2=32 * 3 * window_size,
        disp12MaxDiff=12, uniquenessRatio=10, speckleWindowSize=50,
        speckleRange=32, preFilterCap=63, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )
    right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
    return left_matcher, right_matcher


def undistor_stereo_image(left_frame, right_frame, K1, D1, R1, P1, K2, D2, R2, P2):
    left_map_x, left_map_y = cv2.initUndistortRectifyMap(K1, D1, R1, P1, (IMG_WIDTH, IMG_HEIGHT), cv2.CV_32FC1)
    left_rectified = cv2.remap(left_frame, left_map_x, left_map_y, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)
    right_map_x, right_map_y = cv2.initUndistortRectifyMap(K2, D2, R2, P2, (IMG_WIDTH, IMG_HEIGHT), cv2.CV_32FC1)
    right_rectified = cv2.remap(right_frame, right_map_x, right_map_y, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)
    return left_rectified, right_rectified


def disparity_map(grayscale_left_rectified, grayscale_right_rectified, left_matcher, right_matcher):
    displ = left_matcher.compute(grayscale_left_rectified, grayscale_right_rectified)  # .astype(np.float32)/16
    dispr = right_matcher.compute(grayscale_right_rectified, grayscale_left_rectified)  # .astype(np.float32)/16
    return displ, dispr


def filter_disparity_map(grayscale_image, int16_disp_left, int_16_disp_right, filter):
    filteredImg = filter.filter(int16_disp_left, grayscale_image, None, int_16_disp_right)  # important to put "imgL" here
    filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX)
    return np.uint8(filteredImg)


def left_disparity_from_stereo_image(left_image, right_image, left_matcher, right_matcher, left_filter,
                                K1, D1, R1, P1, K2, D2, R2, P2):
    left_rectified, right_rectified = undistor_stereo_image(left_image, right_image, K1, D1, R1, P1, K2, D2, R2, P2)
    gray_rectified_left = cv2.cvtColor(left_rectified, cv2.COLOR_BGR2GRAY)
    gray_rectified_right = cv2.cvtColor(right_rectified, cv2.COLOR_BGR2GRAY)
    displ, dispr = disparity_map(gray_rectified_left, gray_rectified_right, left_matcher, right_matcher)
    displ = np.int16(displ)
    dispr = np.int16(dispr)
    filtered_left_depth = filter_disparity_map(gray_rectified_left, displ, dispr, left_filter)
    return filtered_left_depth


