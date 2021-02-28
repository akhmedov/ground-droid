import cv2
import numpy as np
import requests
import matplotlib.pyplot as plt

from constants import EDGE_STATION_IP, LEFT_CAMERA_PORT, RIGHT_CAMERA_PORT, \
    MOTOR_CTL_PORT, IMG_WIDTH, IMG_HEIGHT, STREAMING_FPS, EXTRINSIC
from calibration_store import load_stereo_coefficients
from slam import init_image_separator, init_stereo_matchers, left_disparity_from_stereo_image

left_url = r'http://192.168.31.101:5000/left_photo'
right_url = r'http://192.168.31.101:5000/right_photo'


def read_image(url):
    resp = requests.get(url, stream=True).raw
    image = np.asarray(bytearray(resp.read()), dtype="uint8")
    return cv2.imdecode(image, cv2.IMREAD_COLOR)


def show_stereo_image(left, right):
    separator = init_image_separator()
    print(separator.shape)
    print(left.shape)
    print(right.shape)
    image = np.hstack((left, separator, right))
    cv2.imshow('image', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def concatenate_image_and_distance_map(bgr_image, distance_map):
    distance_map[np.isnan(distance_map)] = 0
    # distance_map = cv2.normalize(distance_map, distance_map, 0, 255, cv2.NORM_MINMAX)
    distance_map = np.int16(distance_map)
    # print(distance_map.shape)
    distance_map = cv2.cvtColor(distance_map[..., np.newaxis], cv2.COLOR_GRAY2RGB)
    separator = init_image_separator()
    image = np.hstack((bgr_image, separator, distance_map))
    cv2.imshow('image', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def show_distance_map(lmbda=80000, sigma=1.3):
    left_image = read_image(left_url)
    right_image = read_image(right_url)
    show_stereo_image(left_image, right_image)
    K1, D1, K2, D2, R, T, E, F, R1, R2, P1, P2, Q = load_stereo_coefficients(EXTRINSIC)
    left_matcher, right_matcher = init_stereo_matchers()
    wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
    wls_filter.setLambda(lmbda)
    wls_filter.setSigmaColor(sigma)
    disparity = left_disparity_from_stereo_image(left_image, right_image, left_matcher, right_matcher,
                                                 wls_filter, K1, D1, R1, P1, K2, D2, R2, P2)

    focal_fength = 1000 * K1[2, 2]
    baseline = np.linalg.norm(T)
    distance = focal_fength * baseline / disparity
    plt.imshow(distance, interpolation='none')
    plt.show()

    concatenate_image_and_distance_map(left_image, distance)


show_distance_map()
