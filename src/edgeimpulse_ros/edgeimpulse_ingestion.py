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
import requests
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
        self._latest_img = None
        self._ingestion_event = threading.Event()
        self._shutdown = threading.Event()

        self._frames_received = 0
        self._uploads_sent = 0
        self._uploads_failed = 0
        self._last_error = ''

        self._create_subscription()

        self._worker = threading.Thread(target=self._worker_loop, name='ei_ingestion_upload',
                                        daemon=True)
        self._worker.start()

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
        self._trigger_sub = self.create_subscription(String, 'trigger',
                                                     self._on_trigger,
                                                     self._resolve_qos('reliable'))
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
        """Store the newest received frame for on-demand ingestion."""
        with self._frame_lock:
            self._latest_img = msg
        with self._stats_lock:
            self._frames_received += 1

    def _worker_loop(self):
        """For sending the latest frame to Edge Impulse in a background thread."""
        while not self._shutdown.is_set():
            if not self._ingestion_event.wait(timeout=0.2):
                continue
            self._ingestion_event.clear()
            while not self._shutdown.is_set():
                with self._frame_lock:
                    msg = self._latest_msg
                    self._latest_msg = None
                if msg is None:
                    break
                self._send_to_edge_impulse(msg)

    def _on_trigger(self, msg):
        """Parse a trigger message and upload the latest frame to Edge Impulse."""
        try:
            items = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as exc:
            self._record_error(f'invalid trigger payload: {exc}')
            return

        if not isinstance(items, dict):
            self._record_error('trigger payload must be a JSON object')
            return

        with self._frame_lock:
            latest_img = self._latest_img
        if latest_img is None:
            self._record_error('trigger received but no image has arrived yet')
            return

        category = items.get('category', 'training')
        label = items.get('label')
        filename = items.get('filename')
        self.get_logger().info(
            f'Trigger received: category={category} label={label} filename={filename}')
        with self._frame_lock:
            self._latest_msg = {'img': latest_img,
                                'category': category,
                                'label': label,
                                'filename': filename}
        self._ingestion_event.set()

    # ---------------------------------------------------------------- pipeline

    def _send_to_edge_impulse(self, msg):
        """Upload a single frame to the Edge Impulse ingestion API."""
        img = msg['img']
        category = msg['category']
        label = msg['label']
        filename = msg['filename']

        if category not in INGEST_CATEGORIES:
            self._record_error(f'invalid ingestion category "{category}"')
            return False

        try:
            bgr = (image_utils.compressed_to_bgr(img) if self._compressed
                   else image_utils.ros_image_to_bgr(img))
        except Exception as exc:
            self._record_error(f'image decode failed: {exc}')
            return False

        ok, buffer = cv2.imencode('.jpg', bgr)
        if not ok:
            self._record_error('failed to encode frame as JPEG')
            return False

        name = f'{filename or "frame"}-{int(time.time())}.jpg'
        files = {'data': (name, io.BytesIO(buffer.tobytes()), 'image/jpeg')}
        headers = {'x-api-key': self._api_key}
        if label:
            headers['x-label'] = label
        url = EI_INGEST_URL + f"{category}/files"

        try:
            resp = requests.post(url, files=files, headers=headers, timeout=30)
        except requests.RequestException as exc:
            self._record_error(f'ingestion request failed: {exc}')
            return False

        with self._stats_lock:
            if 200 <= resp.status_code < 300:
                self._uploads_sent += 1
                self._last_error = ''
            else:
                self._uploads_failed += 1

        if 200 <= resp.status_code < 300:
            self.get_logger().info(f'Uploaded frame "{name}" to Edge Impulse ({category})')
            return True

        self._record_error(f'ingestion upload failed ({resp.status_code}): {resp.text}')
        return False

    def _record_error(self, message):
        """Log (throttled) and remember the most recent processing error."""
        self.get_logger().error(message, throttle_duration_sec=2.0)
        with self._stats_lock:
            self._last_error = message

    def _publish_diagnostics(self):
        """Publish a DiagnosticArray summarising ingestion throughput and health."""
        with self._stats_lock:
            frames = self._frames_received
            sent = self._uploads_sent
            failed = self._uploads_failed
            last_error = self._last_error

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
            status.message = f'{sent} uploaded, {failed} failed'
        status.values = [
            KeyValue(key='frames_received', value=str(frames)),
            KeyValue(key='uploads_sent', value=str(sent)),
            KeyValue(key='uploads_failed', value=str(failed)),
        ]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(status)
        self._diag_pub.publish(array)

    # ---------------------------------------------------------------- shutdown

    def destroy_node(self):
        """Stop the worker thread and the ingestion process, then tear down."""
        self._shutdown.set()
        self._ingestion_event.set()
        worker = getattr(self, '_worker', None)
        if worker is not None and worker.is_alive():
            worker.join(timeout=30.0)
        super().destroy_node()


def main(args=None):
    """Console-script entry point for the ``edgeimpulse_ingestion`` executable."""
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
