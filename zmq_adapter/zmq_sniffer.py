#!/usr/bin/env python3

import zmq
import json
import time
from adapter import Adapter

CONFIG_NAME = "zmq_adapter_configuration"
STREAM_RATE = 5


class Mime:
    numeric = "numeric"
    text = "text"
    json = "json"
    geolocation = "geolocation"


class ZMQSniffer(Adapter):
    def __init__(self, is_daemon=False):
        super().__init__(zmq.SUB, is_daemon=is_daemon)

        self.start()

    def _load_config(self):
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.socket.setsockopt(zmq.TCP_KEEPALIVE, Adapter.config.socket.TCP_KEEPALIVE)
        self.socket.setsockopt(
            zmq.TCP_KEEPALIVE_CNT, Adapter.config.socket.TCP_KEEPALIVE_CNT
        )
        self.socket.setsockopt(
            zmq.TCP_KEEPALIVE_IDLE, Adapter.config.socket.TCP_KEEPALIVE_IDLE
        )
        self.socket.setsockopt(
            zmq.TCP_KEEPALIVE_INTVL, Adapter.config.socket.TCP_KEEPALIVE_INTVL
        )
        self.socket.setsockopt(zmq.LINGER, Adapter.config.socket.LINGER)
        self.socket.setsockopt(zmq.TCP_MAXRT, Adapter.config.socket.TCP_MAXRT)

        for i in range(
            Adapter.config.general.port_start, Adapter.config.general.port_end
        ):
            try:
                self.socket.connect(f"tcp://localhost:{i}")
            except zmq.ZMQError:
                print(f"Connect to tcp://localhost:{i}, error")

    def upload_agent(self, key: str, mime: str, message):
        if key in Adapter.config.blacklist:
            return
        post_mime, message = self.mime_method(mime, message)
        post_mime(key, message)

    def mime_method(self, mime: str, message):
        try:
            if Mime.json in mime:
                return self.fclient.post_json, json.loads(message)
            elif Mime.numeric in mime:
                return self.fclient.post_numeric, float(message)
            elif Mime.text in mime:
                return self.fclient.post_text, str(message)
            else:
                print("Mime isn't supported")
        except json.JSONDecodeError:
            print("Error parsing JSON message.")
        except ValueError as e:  #
            print("Value error: %s" % e)

    def _run(self):

        [key, mime, message] = self.socket.recv_multipart()
        key, mime, message = (
            key.decode("utf-8"),
            mime.decode("utf-8"),
            message.decode("utf-8"),
        )

        time.sleep(2 / STREAM_RATE)
        self.upload_agent(key, mime, message)
        print(f">> {key} ({mime}) : {message}")
