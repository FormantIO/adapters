from adapter import Adapter
from threading import Lock
from zmq_sniffer import Mime
import zmq

MESSAGE_TYPE = "command"


class CommandSniffer(Adapter):
    def __init__(self, is_daemon=False):
        super().__init__(zmq.PUB, is_daemon=is_daemon)
        self.mutex = Lock()

        self.start()

    def _load_config(self):
        self.fclient.register_command_request_callback(
            self._command_callback, self.config.commands
        )
        self.socket.bind(
            "tcp://*:{}".format(self.config.command_port)
        )  # replace with PORT from config

    def _command_callback(self, request):
        with self.mutex:
            self.socket.send_multipart(
                [
                    bytes(MESSAGE_TYPE, encoding="utf8"),
                    bytes(Mime.text, encoding="utf8"),
                    bytes(request.command, encoding="utf8"),
                    bytes(request.text, encoding="utf8"),
                ]
            )
