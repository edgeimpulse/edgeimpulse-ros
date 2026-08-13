"""
Edge Impulse ingestion node: image topic in, labelled samples uploaded out.

This node subscribes to a standard ``sensor_msgs/Image`` (or
``CompressedImage``) topic and keeps the latest frame in memory. When a
``std_msgs/String`` JSON trigger message arrives on the ``trigger`` topic, the
latest frame is uploaded to the Edge Impulse ingestion API as a training,
testing or split sample.
"""

from __future__ import annotations

import io
import json
import sys
import threading
import time

import cv2
import requests

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from edgeimpulse_ros import image_utils
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


EI_INGEST_URL = 'https://ingestion.edgeimpulse.com/api/'
INGEST_CATEGORIES = ('training', 'testing', 'split')


class EdgeImpulseIngestion(Node):
    """ROS 2 node that uploads triggered camera frames to Edge Impulse."""

    def __init__(self, **kwargs):
        """Declare parameters and wire up ROS entities."""
        super().__init__('edgeimpulse_ingestion', **kwargs)

        self._api_key = self._declare('api_key', '',
                                       'Edge Impulse ingestion API key')
        self._image_topic = self._declare('image_topic', 'image',
                                           'Input image topic to subscribe to')
        self._transport = self._declare('image_transport', 'raw',
                                         'Image transport: "raw" or "compressed"')
        self._qos_name = self._declare('image_qos', 'sensor_data',
                                        'Subscriber QoS: sensor_data | reliable | default')
        self._publish_diag = bool(self._declare('publish_diagnostics', True,
                                                 'Publish diagnostic_msgs/DiagnosticArray'))
        diag_period = float(self._declare('diagnostic_period', 1.0,
                                           'Diagnostics publish period in seconds'))

        if not self._api_key:
            raise RuntimeError(
                'Parameter "api_key" must be set to a valid Edge Impulse ingestion API key')

        self._compressed = str(self._transport).lower() == 'compressed'

        self._setup_publishers()

        self._frame_lock = threading.Lock()
        self._stats_lock = threading.Lock()
        self._latest_msg = None

        self._frames_received = 0
        self._uploads_sent = 0
        self._uploads_failed = 0
        self._last_error = ''

        self._create_subscription()

        if self._publish_diag and diag_period > 0.0:
            self._diag_timer = self.create_timer(diag_period, self._publish_diagnostics)

        self.get_logger().info('Ingestion node ready to receive triggers')

    def _declare(self, name, default, description):
        """Declare a described parameter and return its initial value."""
        self.declare_parameter(name, default, ParameterDescriptor(description=description))
        return self.get_parameter(name).value

    def _setup_publishers(self):
        """Create the diagnostics publisher, if enabled."""
        self._diag_pub = None
        if self._publish_diag:
            self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

    def _create_subscription(self):
        """Subscribe to the configured image topic and the trigger topic."""
        qos = self._resolve_qos(self._qos_name)
        if self._compressed:
            topic = f'{self._image_topic}/compressed'
            self._sub = self.create_subscription(CompressedImage, topic,
                                                 self._on_image, qos)
        else:
            topic = self._image_topic
            self._sub = self.create_subscription(Image, topic, self._on_image, qos)
        self._trigger_sub = self.create_subscription(String, 'trigger', self._on_trigger, qos)
        self.get_logger().info(
            f'Subscribed to "{topic}" ({self._transport}, qos={self._qos_name})')

    @staticmethod
    def _resolve_qos(name):
        """Translate a QoS preset name into a QoSProfile."""
        if str(name).lower() == 'sensor_data':
            return qos_profile_sensor_data
        return QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)

    # -------------------------------------------------------------- callbacks

    def _on_image(self, msg):
        """Store the newest frame, dropping any previous unprocessed one."""
        with self._frame_lock:
            if self._latest_msg is not None:
                self._pending_drops += 1
            self._latest_msg = msg
        self._frame_event.set()

    def _on_trigger(self, msg):
        """Trigger the ingestion of the latest frame."""
        # trigger should container
        # category
        # optional label
        # filename
        # parse the trigger message and send the latest frame to edge impulse
        items = json.loads(msg.data)

        if self._latest_msg is not None:
            print(f"Trigger received, ingesting frame at {self._latest_msg.header.stamp.sec}.{self._latest_msg.header.stamp.nanosec}")
            print(f"Trigger items: {items}")
            self._send_to_edge_impulse(self._latest_msg, category=items.get('category', 'training'), label=items.get('label', None), filename=items.get('filename', None))
        if msg.data:
            self._frame_event.set()

    # ---------------------------------------------------------------- pipeline

    def _send_to_edge_impulse(self, msg, category='training', label=None, filename=None):
        """Send the image to Edge Impulse ingestion endpoint."""
        bgr = None

        if category not in ['training', 'testing', 'split']:
            return ({'status': 'error', 'message': 'Invalid category'}), 400

        try:
            bgr = (image_utils.compressed_to_bgr(msg) if self._compressed
                   else image_utils.ros_image_to_bgr(msg))
        except Exception as exc:
            self._record_error(f'image decode failed: {exc}')
            return

        # Encode the latest frame as JPEG
        ok, buffer = cv2.imencode('.jpg', bgr)
        if not ok:
            return ({'status': 'error', 'message': 'Failed to encode JPEG'}), 500

        filename = f"{filename}-{int(time.time())}.jpg"
        files = {'data': (filename, io.BytesIO(buffer.tobytes()), 'image/jpeg')}
        headers = {'x-api-key': self._api_key}
        url = EI_INGEST_URL + f"{category}/files"
        resp = requests.post(url, files=files, headers=headers, params={'filename': filename}, timeout=30)

        if 200 <= resp.status_code < 300:
            return ({'status': 'success'})

        return ({'status': 'error', 'message': resp.text, 'status_code': resp.status_code}), resp.status_code

    def _record_error(self, message):
        """Log (throttled) and remember the most recent processing error."""
        self.get_logger().error(message, throttle_duration_sec=2.0)
        with self._stats_lock:
            self._last_error = message

    def _publish_diagnostics(self):
        """Publish a DiagnosticArray summarising throughput and model health."""
        now = time.monotonic()
        with self._stats_lock:
            frames = self._frames
            drops = self._drops
            last_error = self._last_error
            window = frames - self._diag_last_frames
            elapsed = now - self._diag_last_time
            self._diag_last_frames = frames
            self._diag_last_time = now
        fps = (window / elapsed) if elapsed > 0 else 0.0

        status = DiagnosticStatus()
        status.name = f'{self.get_name()}: ingestion'
        status.hardware_id = 'edge_impulse_ingestion'
        if last_error:
            status.level = DiagnosticStatus.ERROR
            status.message = last_error
        elif frames == 0:
            status.level = DiagnosticStatus.WARN
            status.message = 'waiting for images'
        else:
            status.level = DiagnosticStatus.OK
            status.message = f'{fps:.1f} FPS'
        status.values = [
            KeyValue(key='frames', value=str(frames)),
            KeyValue(key='dropped_frames', value=str(drops))
        ]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(status)
        self._diag_pub.publish(array)

def main(args=None):
    """Console-script entry point for the ``edgeimpulse_detector`` executable."""
    rclpy.init(args=args)
    node = None
    try:
        node = EdgeImpulseIngestion()
    except Exception as exc:  # noqa: BLE001 - surface any startup failure cleanly
        print(f'[edgeimpulse_ingestion] failed to start: {exc}', file=sys.stderr)
        rclpy.try_shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
