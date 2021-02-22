import zmq
import pickle
import time
import sys
import argparse


class ZmqMotor:

    def __init__(self, server_ip, server_port):
        context = zmq.Context()
        self.socket = context.socket(zmq.REQ)
        self.socket.connect('tcp://%s:%d' % (server_ip, server_port))

    def native(self, speed_left: float, speed_right: float):
        data = 'native', speed_left, speed_right
        self._set(data)

    def forward(self, speed: float):
        data = 'forward', speed
        self._set(data)

    def backward(self, speed: float):
        data = 'backward', speed
        self._set(data)

    def left(self, speed: float):
        data = 'left', speed
        self._set(data)

    def right(self, speed: float):
        data = 'right', speed
        self._set(data)

    def stop(self):
        data = 'stop', 0
        self._set(data)

    # def _get(self):
    #     self.socket.send(pickle.dumps(('get', None)))
    #     return pickle.loads(self.socket.recv())

    def _set(self, data):
        body = pickle.dumps(data)
        self.socket.send(body)
        return self.socket.recv() == b'ok'
