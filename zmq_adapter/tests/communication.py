import json
from inspect import signature
from threading import Thread, Lock

import zmq

context = zmq.Context()
port_start = 42000
port_end = 42100


class Mime:
    Json = bytes("application/json", encoding="utf8")


class Publisher:
    def __init__(self):
        self.socket = context.socket(zmq.PUB)
        self.mutex = Lock()
        bound = False
        self.port = port_start
        while not bound:

            try:
                self.socket.bind("tcp://*:{}".format(self.port))
                bound = True
            except zmq.ZMQError:
                self.port += 1

            if self.port == port_end:
                raise OSError("no port available")

    def __del__(self):
        self.socket.close()
        del self.socket

    def publish(self, message_type, mime_type, message):
        with self.mutex:
            self.socket.send_multipart(
                [
                    bytes(message_type, encoding="utf8"),
                    bytes(mime_type, encoding="utf8"),
                    bytes(json.dumps(message), encoding="utf8"),
                ]
            )


class Subscriber(Thread):
    def __init__(
        self, message_type, message_mime, callback, is_daemon=False, ip="localhost"
    ):
        self.callback = callback

        self.mime = message_mime

        self.socket = context.socket(zmq.SUB)

        self.socket.setsockopt_string(zmq.SUBSCRIBE, message_type)

        # http://api.zeromq.org/4-0:zmq-setsockopt
        self.socket.setsockopt(zmq.TCP_KEEPALIVE, 1)
        self.socket.setsockopt(zmq.TCP_KEEPALIVE_CNT, 2)
        self.socket.setsockopt(zmq.TCP_KEEPALIVE_IDLE, 1)
        self.socket.setsockopt(zmq.TCP_KEEPALIVE_INTVL, 1)
        self.socket.setsockopt(zmq.LINGER, 5000)
        self.socket.setsockopt(zmq.TCP_MAXRT, 500)

        for i in range(port_start, port_end):
            try:
                self.socket.connect(("tcp://" + ip + ":{}").format(i))
            except zmq.ZMQError:
                print("Connect to", ("tcp://" + ip + ":{}"), " error".format(i))

        Thread.__init__(
            self, name="Subscriber_{}".format(message_type), daemon=is_daemon
        )

        self.do_run = True
        self.start()

    def stop(self):
        self.do_run = False

    def __del__(self):
        self.do_run = False
        self.join(0.5)
        self.socket.close()
        del self.socket

    def run(self):
        while self.do_run:
            try:
                [message_type, message_mime, message] = self.socket.recv_multipart()
                if message_mime == self.mime:
                    decoded = message.decode("utf8")
                    if len(signature(self.callback).parameters) == 1:
                        self.callback(decoded)
                    else:
                        self.callback(message_type, decoded)
            except ValueError:
                pass  # too few or too much arguments, not interested
