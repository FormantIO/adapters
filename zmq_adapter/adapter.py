#!/usr/bin/env python3

from formant.sdk.agent.v1 import Client as FormantClient
from formant.sdk.agent.adapter_utils import JsonSchemaValidator
from threading import Thread, Lock
from util.config import Config
import zmq
import time

CONFIG_NAME = "zmq_adapter_configuration"


class Adapter(Thread):
    fclient = FormantClient()
    config: Config = None
    config_lock = Lock()
    is_config_initialized = False

    def __init__(self, socket_context, is_daemon=False):
        self.context = zmq.Context()
        self.socket = self.context.socket(socket_context)
        self.is_config_loaded = False
        self.configuration_validator = JsonSchemaValidator(
            self.fclient,
            CONFIG_NAME,
            Adapter.initialize_config,
            validate=False,
        )
        Thread.__init__(self, daemon=is_daemon)

        self.do_run = True

    def run(self):
        while self.do_run:
            time.sleep(0.1)
            if not Adapter.is_config_initialized:
                continue

            if not self.is_config_loaded:
                self._load_config()
                self.is_config_loaded = True

            self._run()

    def _load_config(self):
        pass

    def _run(self):
        pass

    def __del__(self):
        self.do_run = False
        self.join(0.5)
        self.socket.close()
        del self.socket

    @classmethod
    def initialize_config(cls, config_raw):
        with cls.config_lock:
            is_config_zmq = CONFIG_NAME in config_raw

            if is_config_zmq and not cls.is_config_initialized:
                cls.config = Config(config_raw[CONFIG_NAME])
