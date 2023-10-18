import json
from typing import List, Dict


class GeneralConfig:
    def __init__(self, config_dict: Dict):
        self.port_start: int = config_dict["port_start"]
        self.port_end: int = config_dict["port_end"]


class SocketConfig:
    def __init__(self, config_dict: Dict):
        self.TCP_KEEPALIVE: int = config_dict["TCP_KEEPALIVE"]
        self.TCP_KEEPALIVE_CNT: int = config_dict["TCP_KEEPALIVE_CNT"]
        self.TCP_KEEPALIVE_IDLE: int = config_dict["TCP_KEEPALIVE_IDLE"]
        self.TCP_KEEPALIVE_INTVL: int = config_dict["TCP_KEEPALIVE_INTVL"]
        self.LINGER: int = config_dict["LINGER"]
        self.TCP_MAXRT: int = config_dict["TCP_MAXRT"]


class Config:
    def __init__(self, zmq_config: Dict):
        self.zmq_config = zmq_config

        self.general: GeneralConfig = GeneralConfig(self.zmq_config["general_config"])
        self.socket: SocketConfig = SocketConfig(self.zmq_config["socket_config"])
        self.blacklist: List[str] = self.zmq_config["blacklist"]
        self.commands: List[str] = self.zmq_config["commands"]
        self.command_port: int = self.zmq_config["command_port"]

    def _load_json(self, file_name):
        file = open(file_name, "r").read()
        return json.loads(file)
