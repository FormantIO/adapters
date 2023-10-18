from communication import Publisher
from formant.sdk.cloud.v2 import Client
from formant.sdk.cloud.v2.src.resources.queries import (
    Query,
    StreamDataListResponse,
    StreamData,
)
import unittest
import random
import datetime
import os
import subprocess
import time
import argparse

DEVICE_ID = os.environ["DEVICE_ID"]


def initial_data_point(parsed_response: StreamDataListResponse, mime_type: str):
    stream_data: StreamData = parsed_response.items[0]
    stream_data_items = stream_data.points
    initial_item = stream_data_items[0]
    return getattr(initial_item, mime_type)


class TestPublish(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestPublish, self).__init__(*args, **kwargs)
        self.publisher = Publisher()
        self.fclient = Client()

    def test_numeric(self):
        stream_name = "numeric_test_f4b231"
        message = random.randint(1, 100)
        start_time = datetime.datetime.now(datetime.timezone.utc)
        self.publisher.publish(stream_name, "numeric", message)
        time.sleep(2)
        end_time = datetime.datetime.now(datetime.timezone.utc)
        query = Query(
            start=start_time, end=end_time, device_ids=[DEVICE_ID], names=[stream_name]
        )
        result = self.fclient.query.queries.query(query)
        result_message = initial_data_point(result.parsed, "numeric")

        self.assertEqual(message, result_message)

    def test_text(self):
        stream_name = "text_test_f4b231"
        message = "sample_text"
        start_time = datetime.datetime.now(datetime.timezone.utc)
        self.publisher.publish(stream_name, "text", message)
        time.sleep(2)
        end_time = datetime.datetime.now(datetime.timezone.utc)
        query = Query(
            start=start_time, end=end_time, device_ids=[DEVICE_ID], names=[stream_name]
        )
        result = self.fclient.query.queries.query(query)
        result_message = initial_data_point(result.parsed, "text")

        self.assertEqual(message, eval(result_message))

    def test_json(self):
        stream_name = "json_test_f4b231"
        message = '{"sample":"json"}'

        start_time = datetime.datetime.now(datetime.timezone.utc)
        self.publisher.publish(stream_name, "json", message)
        time.sleep(2)
        end_time = datetime.datetime.now(datetime.timezone.utc)
        query = Query(
            start=start_time, end=end_time, device_ids=[DEVICE_ID], names=[stream_name]
        )
        result = self.fclient.query.queries.query(query)
        result_message = initial_data_point(result.parsed, "json")

        self.assertEqual(message, result_message)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Python ZMQ Adapter Test")
    parser.add_argument("--local", action="store_true", help="Enable local test")

    args = parser.parse_args()

    if args.local:
        adapter = subprocess.Popen(
            ["python", "main.py"], stdout=subprocess.PIPE
        )
        unittest.main()
        adapter.kill()
    else:
        unittest.main()
