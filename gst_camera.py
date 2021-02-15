import traitlets
import atexit
import cv2
import threading
import numpy as np
import os
import time
import sys
import zmq
import time


def recv_image(socket, dtype, shape):
    data = socket.recv()
    buf = memoryview(data)
    array = np.frombuffer(buf, dtype=dtype)
    return array.reshape(shape)


class CameraBase(traitlets.HasTraits):
    
    value = traitlets.Any()
    
    @staticmethod
    def instance(*args, **kwargs):
        raise NotImplementedError
    
    def widget(self):
        if hasattr(self, '_widget'):
            return self._widget   # cache widget, so we don't duplicate links
        from ipywidgets import Image
        from jetbot.image import bgr8_to_jpeg
        image = Image()
        traitlets.dlink((self, 'value'), (image, 'value'), transform=bgr8_to_jpeg)
        self._widget = image
        return image


class ZmqCamera(CameraBase):
    
    value = traitlets.Any(value=np.zeros((224, 224, 3), dtype=np.uint8), default_value=np.zeros((224, 224, 3), dtype=np.uint8))
    
    def __init__(self, ip='192.168.31.101', port=1807, *args, **kwargs):
        self.value = np.zeros((224, 224, 3), dtype=np.uint8)  # set default image
        super().__init__(self, *args, **kwargs)
        self._running = False
        self._ip = ip
        self._port = port
        self._image_shape = (224, 224, 3)
        self._image_dtype = np.uint8
        self.start()
        atexit.register(self.stop)
        
    def __del__(self):
        self.stop()
        
    def _run(self):
        context = zmq.Context()
        self.socket = context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)  # last msg only.
        self.socket.setsockopt(zmq.SUBSCRIBE, b'') # all topics
        self.socket.connect("tcp://%s:%d" % (self._ip, self._port))
        while self._running:
            image = recv_image(self.socket, self._image_dtype, self._image_shape)
            self.value = image
        self.socket.close()
            
    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run)
        self._thread.start()
        
    def stop(self):
        if not self._running:
            return
        self._running = False
        self._thread.join()
        
    @staticmethod
    def instance(*args, **kwargs):
        return ZmqCamera(*args, **kwargs)


class OpenCvGstCamera(CameraBase):
    
    value = traitlets.Any()
    
    # config
    width = traitlets.Integer(default_value=224).tag(config=True)
    height = traitlets.Integer(default_value=224).tag(config=True)
    fps = traitlets.Integer(default_value=30).tag(config=True)
    capture_width = traitlets.Integer(default_value=816).tag(config=True)
    capture_height = traitlets.Integer(default_value=616).tag(config=True)
    sensor_id = traitlets.Integer(default_value=0).tag(config=True)

    def __init__(self, *args, **kwargs):
        self.value = np.empty((self.height, self.width, 3), dtype=np.uint8)
        super().__init__(self, *args, **kwargs)

        try:
            self.cap = cv2.VideoCapture(self._gst_str(), cv2.CAP_GSTREAMER)

            re, image = self.cap.read()

            if not re:
                raise RuntimeError('Could not read image from camera.')

            self.value = image
            self.start()
        except:
            self.stop()
            raise RuntimeError(
                'Could not initialize camera.  Please see error trace.')

        atexit.register(self.stop)

    def _capture_frames(self):
        while True:
            re, image = self.cap.read()
            if re:
                self.value = image
            else:
                break
                
    def _gst_str(self):
        return 'nvarguscamerasrc sensor_id=%d sensor-mode=3 ! video/x-raw(memory:NVMM), width=%d, height=%d, format=(string)NV12, framerate=(fraction)%d/1 ! nvvidconv ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! videoconvert ! appsink' % (
                self.sensor_id, self.capture_width, self.capture_height, self.fps, self.width, self.height)
    
    def start(self):
        if not self.cap.isOpened():
            self.cap.open(self._gst_str(), cv2.CAP_GSTREAMER)
        if not hasattr(self, 'thread') or not self.thread.isAlive():
            self.thread = threading.Thread(target=self._capture_frames)
            self.thread.start()

    def stop(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        if hasattr(self, 'thread'):
            self.thread.join()
            
    def restart(self):
        self.stop()
        self.start()
        
    @staticmethod
    def instance(*args, **kwargs):
        return OpenCvGstCamera(*args, **kwargs)


# left = OpenCvGstCamera(sensor_id=0)
# left.start()
# right = OpenCvGstCamera(sensor_id=1)
# right.start()

# frames = 0
# all_frames = 0 
# timestump = time.time()
# left_old = left.value
# right_old = right.value

# while True:
#     if not (left.value==left_old).all() and not (right.value==right_old).all():
#         frames += 1
#         all_frames += 1
#         if frames == 5:
#             print(frames/(time.time()-timestump))
#             frames = 0
#             timestump = time.time()
#         left_old = left.value
#         right_old = right.value

# left = ZmqCamera(port=1807)
# right = ZmqCamera(port=1808)
# frames = 0
# all_frames = 0 
# timestump = time.time()
# left_old = left.value
# right_old = right.value

# while True:
#     if not (left.value==left_old).all() and not (right.value==right_old).all():
#         frames += 1
#         all_frames += 1
#         if frames == 5:
#             print(frames/(time.time()-timestump))
#             frames = 0
#             timestump = time.time()
#         left_old = left.value
#         right_old = right.value
