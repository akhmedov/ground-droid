import zmq
import pickle
import time
import sys
import argparse
from robot import Robot


class ZmqMotorPublisher:

    def __init__(self, server_ip, server_port):

        self.last_update = time.time()

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind('tcp://%s:%d' % (server_ip, server_port))
        self.robot = Robot()

    def listening_loop(self):

        while True:

            if time.time() - self.last_update > 0.5:
                # TODO: handle dublicative sendings
                self.robot.stop()

            try:
                body = self.socket.recv(flags=zmq.NOBLOCK)
                data_tuple = pickle.loads(body)

                if data_tuple[0] == 'native':
                    self.robot.set_motors(data_tuple[1], data_tuple[2])
                    self.last_update = time.time()
                    self.socket.send(b'ok')
                elif data_tuple[0] == 'forward':
                    self.robot.forward(data_tuple[1])
                    self.last_update = time.time()
                    self.socket.send(b'ok')
                elif data_tuple[0] == 'backward':
                    self.robot.backward(data_tuple[1])
                    self.last_update = time.time()
                    self.socket.send(b'ok')
                elif data_tuple[0] == 'left':
                    self.robot.left(data_tuple[1])
                    self.last_update = time.time()
                    self.socket.send(b'ok')
                elif data_tuple[0] == 'right':
                    self.robot.right(data_tuple[1])
                    self.last_update = time.time()
                    self.socket.send(b'ok')
                elif data_tuple[0] == 'stop':
                    self.robot.stop()
                    self.socket.send(b'ok')

                # elif data_tuple[0] == 'get':
                #     result = self.current_command
                #     self.socket.send(pickle.dumps(result))

            except zmq.error.Again:
                try:
                    time.sleep(0.1)
                except KeyboardInterrupt:
                    self.context.term()
                    sys.exit(0)


if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=3434)
    args = parser.parse_args()
    
    motor_publisher = ZmqMotorPublisher(server_ip='*', server_port=args.port)
    motor_publisher.listening_loop()
