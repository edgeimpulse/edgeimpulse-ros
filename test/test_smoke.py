"""Smoke tests: drive the node end-to-end with a mocked Edge Impulse SDK."""

import sys
import time
import types

from edgeimpulse_ros.edgeimpulse_detector import EdgeImpulseDetector
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from vision_msgs.msg import Classification, Detection2DArray


def _model(model_type, labels, has_anomaly=0):
    """Build a minimal Edge Impulse ``init()`` model description."""
    return {
        'model_type': model_type,
        'image_input_width': 32,
        'image_input_height': 32,
        'image_channel_count': 3,
        'labels': labels,
        'has_anomaly': has_anomaly,
        'image_resize_mode': 'squash',
        'threshold': 0.5,
    }


def _install_fake_sdk(model_parameters, classify_result):
    """Register a fake ``edge_impulse_linux`` package in ``sys.modules``."""
    class _Runner:
        def __init__(self, model_path, *args, **kwargs):
            pass

        def init(self, debug=False):
            return {'project': {'owner': 'test', 'name': 'smoke'},
                    'model_parameters': model_parameters}

        def classify(self, features):
            return classify_result

        def stop(self):
            pass

    pkg = types.ModuleType('edge_impulse_linux')
    runner = types.ModuleType('edge_impulse_linux.runner')
    runner.ImpulseRunner = _Runner
    pkg.runner = runner
    sys.modules['edge_impulse_linux'] = pkg
    sys.modules['edge_impulse_linux.runner'] = runner


def _make_image():
    """Build a blank 32x32 bgr8 image message."""
    msg = Image()
    msg.height = 32
    msg.width = 32
    msg.encoding = 'bgr8'
    msg.step = 32 * 3
    msg.data = bytes(32 * 32 * 3)
    return msg


def _capture(model_parameters, classify_result, msg_type, topic):
    """Run the node against a mocked model; return the last message on topic."""
    _install_fake_sdk(model_parameters, classify_result)
    rclpy.init()
    detector = None
    helper = None
    try:
        detector = EdgeImpulseDetector(parameter_overrides=[
            Parameter('model_path', value='dummy.eim'),
            Parameter('image_topic', value='smoke_image'),
            Parameter('publish_diagnostics', value=False),
        ])

        helper = Node('smoke_helper')
        pub = helper.create_publisher(Image, 'smoke_image', 10)
        received = []
        helper.create_subscription(msg_type, topic, received.append, 10)

        executor = SingleThreadedExecutor()
        executor.add_node(detector)
        executor.add_node(helper)

        deadline = time.time() + 10.0
        while time.time() < deadline and not received:
            pub.publish(_make_image())
            executor.spin_once(timeout_sec=0.1)
        return received[-1] if received else None
    finally:
        if detector is not None:
            detector.destroy_node()
        if helper is not None:
            helper.destroy_node()
        rclpy.shutdown()


def test_object_detection_publishes_boxes():
    """An object-detection model republishes a Detection2DArray."""
    result = {'result': {'bounding_boxes': [
        {'label': 'thing', 'value': 0.9, 'x': 4, 'y': 4, 'width': 8, 'height': 8}]}}
    msg = _capture(_model('object_detection', ['thing']), result,
                   Detection2DArray, '/edgeimpulse_detector/detections')
    assert msg is not None, 'no Detection2DArray within 10s'
    assert len(msg.detections) == 1
    assert msg.detections[0].results[0].hypothesis.class_id == 'thing'


def test_classification_publishes_scores():
    """A classification model republishes a Classification message."""
    result = {'result': {'classification': {'cat': 0.8, 'dog': 0.2}}}
    msg = _capture(_model('classification', ['cat', 'dog']), result,
                   Classification, '/edgeimpulse_detector/classification')
    assert msg is not None, 'no Classification within 10s'
    assert [r.class_id for r in msg.results][0] == 'cat'


def test_anomaly_publishes_score():
    """A model with anomaly republishes the max score as Float32."""
    result = {
        'result': {'classification': {'cat': 0.8, 'dog': 0.2}, 'anomaly': 0.42},
    }
    msg = _capture(_model('classification', ['cat', 'dog'], has_anomaly=1),
                   result, Float32, '/edgeimpulse_detector/anomaly')
    assert msg is not None, 'no anomaly Float32 within 10s'
    assert abs(msg.data - 0.42) < 1e-4


def test_visual_anomaly_publishes_grid_boxes():
    """A visual-anomaly (FOMO-AD) model republishes grid cells as detections."""
    result = {'result': {
        'visual_anomaly_grid': [
            {'label': 'anomaly', 'value': 6.0,
             'x': 4, 'y': 4, 'width': 8, 'height': 8}],
        'visual_anomaly_max': 6.0,
        'visual_anomaly_mean': 3.0,
    }}
    msg = _capture(_model('classification', [], has_anomaly=4), result,
                   Detection2DArray, '/edgeimpulse_detector/detections')
    assert msg is not None, 'no Detection2DArray within 10s'
    assert len(msg.detections) == 1
    assert msg.detections[0].results[0].hypothesis.class_id == 'anomaly'
