"""Smoke tests: drive the ingestion node with a mocked HTTP endpoint."""

import json
import time
from unittest.mock import MagicMock, patch

from edgeimpulse_ros.edgeimpulse_ingestion import EdgeImpulseIngestion
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_msgs.msg import String


def _make_image():
    """Build a blank 4x4 bgr8 image message."""
    msg = Image()
    msg.height = 4
    msg.width = 4
    msg.encoding = 'bgr8'
    msg.step = 4 * 3
    msg.data = bytes(4 * 4 * 3)
    return msg


def _run_trigger(trigger_payload):
    """Publish an image then a trigger; return the mocked requests.post call."""
    rclpy.init()
    node = None
    helper = None
    try:
        node = EdgeImpulseIngestion(parameter_overrides=[
            Parameter('api_key', value='ei_test_key'),
            Parameter('image_topic', value='ingest_image'),
            Parameter('publish_diagnostics', value=False),
        ])

        helper = Node('ingestion_smoke_helper')
        image_pub = helper.create_publisher(Image, 'ingest_image', 10)
        trigger_pub = helper.create_publisher(String, 'trigger', 10)

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(helper)

        with patch('edgeimpulse_ros.edgeimpulse_ingestion.requests.post') as post:
            post.return_value = MagicMock(status_code=200, text='ok')

            deadline = time.time() + 10.0
            while time.time() < deadline and node._latest_img is None:
                image_pub.publish(_make_image())
                executor.spin_once(timeout_sec=0.1)

            trigger_pub.publish(String(data=json.dumps(trigger_payload)))
            deadline = time.time() + 10.0
            while time.time() < deadline and not post.called:
                executor.spin_once(timeout_sec=0.1)

            return post
    finally:
        if node is not None:
            node.destroy_node()
        if helper is not None:
            helper.destroy_node()
        rclpy.shutdown()


def test_trigger_uploads_latest_frame_with_label():
    """A trigger with a label uploads the latest frame using the x-label header."""
    post = _run_trigger({'category': 'training', 'label': 'cat', 'filename': 'sample'})
    assert post.called, 'requests.post was not called within 10s'
    _, kwargs = post.call_args
    assert kwargs['headers']['x-label'] == 'cat'
    assert kwargs['headers']['x-api-key'] == 'ei_test_key'
    assert post.call_args[0][0] == 'https://ingestion.edgeimpulse.com/api/training/files'


def test_invalid_trigger_json_does_not_raise():
    """Malformed trigger payloads are handled gracefully, not raised."""
    rclpy.init()
    node = None
    try:
        node = EdgeImpulseIngestion(parameter_overrides=[
            Parameter('api_key', value='ei_test_key'),
            Parameter('publish_diagnostics', value=False),
        ])
        node._on_trigger(String(data='not json'))
        assert 'invalid trigger payload' in node._last_error
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_trigger_with_non_dict_payload_does_not_raise():
    """Trigger payloads that are not JSON objects are handled gracefully, not raised."""
    rclpy.init()
    node = None
    try:
        node = EdgeImpulseIngestion(parameter_overrides=[
            Parameter('api_key', value='ei_test_key'),
            Parameter('publish_diagnostics', value=False),
        ])
        node._on_trigger(String(data=json.dumps(['not', 'a', 'dict'])))
        assert 'trigger payload must be a JSON object' in node._last_error
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
