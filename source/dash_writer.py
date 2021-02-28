import cv2
import numpy as np
import os
import glob
import time
from gst_camera import ZmqCamera
from calibration_store import load_stereo_coefficients
from constants import LEFT_CAMERA_PORT, RIGHT_CAMERA_PORT, \
    IMG_WIDTH, IMG_HEIGHT, STREAMING_FPS, EXTRINSIC

VIDEO_FOLDER = 'dash_video'
VIDEO_DURATION = 60
VIDEOS_TO_STORE = 30

WINDOW_SIZE = 3
left_matcher = cv2.StereoSGBM_create(
    minDisparity=-1,
    numDisparities=5 * 16,  # max_disp has to be dividable by 16 f. E. HH 192, 256
    blockSize=WINDOW_SIZE,
    P1=8 * 3 * WINDOW_SIZE,  # wsize default 3; 5; 7 for SGBM reduced size image;
    P2=32 * 3 * WINDOW_SIZE,
    disp12MaxDiff=12,
    uniquenessRatio=10,
    speckleWindowSize=50,
    speckleRange=32,
    preFilterCap=63,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)
right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)

IMAGE_SEPARATOR = np.zeros(shape=(IMG_WIDTH, 10, 3), dtype=np.uint8)
IMAGE_SEPARATOR[0::5, :] = 255, 255, 255


def depth_map_left(left_frame, right_frame, lmbda=80000, sigma=1.3):
    left_map_x, left_map_y = cv2.initUndistortRectifyMap(K1, D1, R1, P1, (IMG_WIDTH, IMG_HEIGHT), cv2.CV_32FC1)
    left_rectified = cv2.remap(left_frame, left_map_x, left_map_y, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)
    right_map_x, right_map_y = cv2.initUndistortRectifyMap(K2, D2, R2, P2, (IMG_WIDTH, IMG_HEIGHT), cv2.CV_32FC1)
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


def remove_old_videos(video_folder=VIDEO_FOLDER, allowed_number=VIDEOS_TO_STORE):
    files = glob.glob(video_folder + '/*.mp4')
    while len(files) > allowed_number:
        oldest = files.pop(0)
        os.remove(oldest)


K1, D1, K2, D2, R, T, E, F, R1, R2, P1, P2, Q = load_stereo_coefficients(EXTRINSIC)
left_camera = ZmqCamera(ip='127.0.0.1', port=LEFT_CAMERA_PORT, width=IMG_WIDTH, height=IMG_HEIGHT)
right_camera = ZmqCamera(ip='127.0.0.1', port=RIGHT_CAMERA_PORT, width=IMG_WIDTH, height=IMG_HEIGHT)
time.sleep(1)

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
writer_berth = time.time()
video_writer = None
while True:

    if time.time() - writer_berth > VIDEO_DURATION or not video_writer:
        remove_old_videos(video_folder=VIDEO_FOLDER, allowed_number=VIDEOS_TO_STORE)
        video_writer.release()
        writer_berth = time.time()
        writer_file = os.path.join(VIDEO_FOLDER, str(int(writer_berth)) + '.mp4')
        shape = int(1.5 * (2 * IMG_WIDTH + 10)), int(1.5 * IMG_HEIGHT)
        video_writer = cv2.VideoWriter(writer_file, fourcc, STREAMING_FPS, shape)

    depth = depth_map_left(left_camera.value, right_camera.value)
    leftMapX, leftMapY = cv2.initUndistortRectifyMap(K1, D1, R1, P1, (IMG_WIDTH, IMG_HEIGHT), cv2.CV_32FC1)
    left_rectified = cv2.remap(left_camera.value, leftMapX, leftMapY, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT)
    depth = cv2.cvtColor(depth[..., np.newaxis], cv2.COLOR_GRAY2RGB)
    image = np.hstack((left_rectified, IMAGE_SEPARATOR, depth))
    image = cv2.resize(image, (int(image.shape[1] * 1.5), int(image.shape[0] * 1.5)), interpolation=cv2.INTER_AREA)
    video_writer.write(image)
