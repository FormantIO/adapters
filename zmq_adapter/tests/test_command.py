import zmq
import unittest
import argparse
import subprocess

COMMAND_PORT = 4343


class TestPublisherOutput(unittest.TestCase):
    def setUp(self):
        self.context = zmq.Context()
        self.subscriber = self.context.socket(zmq.SUB)
        self.subscriber.connect(f"tcp://localhost:{COMMAND_PORT}")
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, "")

    def tearDown(self):
        # Cleanup function: Close the socket and terminate the ZMQ context
        self.subscriber.close()
        self.context.term()

    def test_command(self):
        expected_message_type = "command"
        expected_mime_type = "text"
        expected_command = "test_cloud_sdk"
        expected_value = ""
        [
            message_type_bytes,
            mime_type_bytes,
            command_bytes,
            value_bytes,
        ] = self.subscriber.recv_multipart()
        message_type = message_type_bytes.decode("utf8")
        mime_type = mime_type_bytes.decode("utf8")
        command = command_bytes.decode("utf8")
        value = value_bytes.decode("utf-8")

        self.assertEqual(message_type, expected_message_type)
        self.assertEqual(mime_type, expected_mime_type)  # Adjust as necessary
        self.assertEqual(command, expected_command)
        self.assertEqual(value, expected_value)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Python ZMQ Adapter Test")
    parser.add_argument("--local", action="store_true", help="Enable local test")

    args = parser.parse_args()

    if args.local:
        adapter = subprocess.Popen(["python", "main.py"], stdout=subprocess.PIPE)
        unittest.main()
        adapter.kill()
    else:
        unittest.main()
