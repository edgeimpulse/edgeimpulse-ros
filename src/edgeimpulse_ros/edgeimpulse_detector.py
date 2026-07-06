"""
Edge Impulse detector node: images in, ``vision_msgs`` out.

This node is deliberately camera-agnostic. It subscribes to a standard
``sensor_msgs/Image`` (or ``CompressedImage``) topic produced by *any* camera
driver, runs an Edge Impulse ``.eim`` model on each frame, and publishes the
result using idiomatic ``vision_msgs`` message types. Source image timestamps
and ``frame_id`` are propagated so downstream TF lookups and sensor fusion keep
working.
"""

from __future__ import annotations

import sys
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from edgeimpulse_ros import conversions, image_utils
from edgeimpulse_ros.model_runner import (
    KIND_ANOMALY,
    KIND_CLASSIFICATION,
    KIND_DETECTION,
    ModelRunner,
)
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32, Header

_RESIZE_ALIASES = {
    'squash': 'squash', 'none': 'squash', 'stretch': 'squash',
    'fit-shortest': 'fit-shortest', 'fit-short': 'fit-shortest',
    'crop': 'fit-shortest', 'cover': 'fit-shortest',
    'fit-longest': 'fit-longest', 'fit-long': 'fit-longest',
    'pad': 'fit-longest', 'contain': 'fit-longest', 'letterbox': 'fit-longest',
}


def _normalize_resize_mode(mode: str, fallback: str = 'fit-shortest') -> str:
    """Map assorted resize-mode spellings onto the three canonical modes."""
    return _RESIZE_ALIASES.get(str(mode).lower().strip(), fallback)


class EdgeImpulseDetector(Node):
    """ROS 2 node wrapping a single Edge Impulse image model."""

    def __init__(self, **kwargs):
        """Declare parameters, load the model and wire up ROS entities."""
        super().__init__('edgeimpulse_detector', **kwargs)

        model_path = self._declare('model_path', '',
                                   'Path to the Edge Impulse .eim model (required)')
        self._image_topic = self._declare('image_topic', 'image',
                                          'Input image topic to subscribe to')
        self._transport = self._declare('image_transport', 'raw',
                                        'Image transport: "raw" or "compressed"')
        self._qos_name = self._declare('image_qos', 'sensor_data',
                                       'Subscriber QoS: sensor_data | reliable | default')
        resize_mode = self._declare('resize_mode', 'auto',
                                    'auto | squash | fit-shortest | fit-longest')
        threshold = float(self._declare('confidence_threshold', -1.0,
                                        'Min confidence to publish; <0 uses model default'))
        self._publish_debug = bool(self._declare('publish_debug_image', False,
                                                 'Publish an annotated debug image'))
        self._overlay_labels = bool(self._declare('overlay_labels', True,
                                                  'Draw labels/scores on the debug image'))
        self._frame_id_override = self._declare('frame_id_override', '',
                                                'Override the source image frame_id if set')
        self._publish_diag = bool(self._declare('publish_diagnostics', True,
                                                'Publish diagnostic_msgs/DiagnosticArray'))
        diag_period = float(self._declare('diagnostic_period', 1.0,
                                          'Diagnostics publish period in seconds'))
        self._warn_on_drop = bool(self._declare('warn_on_drop', False,
                                                'Log a warning when stale frames are dropped'))
        publish_label_info = bool(self._declare('publish_label_info', True,
                                                'Publish a latched vision_msgs/LabelInfo'))

        if not model_path:
            raise RuntimeError('Parameter "model_path" is required (path to the .eim file)')

        self._threshold = None if threshold < 0.0 else threshold
        self._compressed = str(self._transport).lower() == 'compressed'

        # --- Load the model up front so startup fails fast on a bad path. ---
        self._runner = ModelRunner(model_path)
        self._model = self._runner.start()
        self._kinds = self._model.output_kinds
        self._resize_mode = (_normalize_resize_mode(self._model.resize_mode)
                             if str(resize_mode).lower() == 'auto'
                             else _normalize_resize_mode(resize_mode))
        self.get_logger().info(
            f'Loaded model "{self._model.owner}/{self._model.name}" '
            f'type={self._model.model_type or "?"} '
            f'input={self._model.input_width}x{self._model.input_height} '
            f'{"gray" if self._model.grayscale else "rgb"} '
            f'resize={self._resize_mode} has_anomaly={self._model.has_anomaly} '
            f'kinds={sorted(self._kinds)} labels={self._model.labels}')

        if not self._model.is_image_model:
            raise RuntimeError(
                'Loaded model does not expect an image input; this node only '
                'supports Edge Impulse image models.')

        self._setup_publishers(publish_label_info)
        self._publish_metadata()

        # --- Threading / frame handoff state (latest-frame-wins). ---
        self._frame_lock = threading.Lock()
        self._stats_lock = threading.Lock()
        self._latest_msg = None
        self._pending_drops = 0
        self._frame_event = threading.Event()
        self._shutdown = threading.Event()

        self._frames = 0
        self._drops = 0
        self._last_latency_ms = 0.0
        self._ei_timing = {}
        self._last_anomaly = None
        self._last_error = ''
        self._logged_result = False
        self._diag_last_frames = 0
        self._diag_last_time = time.monotonic()

        self._create_subscription()

        self._worker = threading.Thread(target=self._worker_loop, name='ei_inference',
                                        daemon=True)
        self._worker.start()

        if self._publish_diag and diag_period > 0.0:
            self._diag_timer = self.create_timer(diag_period, self._publish_diagnostics)

    # ------------------------------------------------------------------ setup

    def _declare(self, name, default, description):
        """Declare a described parameter and return its initial value."""
        self.declare_parameter(name, default, ParameterDescriptor(description=description))
        return self.get_parameter(name).value

    def _setup_publishers(self, publish_label_info):
        """Create result publishers according to the model's output kinds."""
        results_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                 history=HistoryPolicy.KEEP_LAST)
        latched_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 history=HistoryPolicy.KEEP_LAST,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._detections_pub = None
        self._classification_pub = None
        self._anomaly_pub = None
        if KIND_DETECTION in self._kinds:
            self._detections_pub = self.create_publisher(
                conversions.Detection2DArray, '~/detections', results_qos)
        if KIND_CLASSIFICATION in self._kinds:
            self._classification_pub = self.create_publisher(
                conversions.ClassificationMsg, '~/classification', results_qos)
        if KIND_ANOMALY in self._kinds:
            self._anomaly_pub = self.create_publisher(Float32, '~/anomaly', results_qos)

        self._vision_info_pub = self.create_publisher(
            conversions.VisionInfo, '~/vision_info', latched_qos)
        self._label_info_pub = None
        if publish_label_info and conversions.has_label_info():
            self._label_info_pub = self.create_publisher(
                conversions.LabelInfo, '~/label_info', latched_qos)

        self._debug_pub = None
        if self._publish_debug:
            self._debug_pub = self.create_publisher(Image, '~/debug_image',
                                                    qos_profile_sensor_data)

        self._diag_pub = None
        if self._publish_diag:
            self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

    def _publish_metadata(self):
        """Publish latched label metadata and expose labels as a parameter."""
        # An empty list has no inferable type, so only declare when populated.
        if self._model.labels:
            self.declare_parameter('class_labels', list(self._model.labels))
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._frame_id_override

        database_location = f'{self.get_fully_qualified_name()}/class_labels'
        self._vision_info_pub.publish(
            conversions.make_vision_info(
                f'edge_impulse:{self._model.model_type or "image"}',
                database_location, header))

        if self._label_info_pub is not None:
            label_info = conversions.make_label_info(
                self._model.labels, self._model.default_threshold, header)
            if label_info is not None:
                self._label_info_pub.publish(label_info)

    def _create_subscription(self):
        """Subscribe to the configured image topic with the requested QoS."""
        qos = self._resolve_qos(self._qos_name)
        if self._compressed:
            topic = f'{self._image_topic}/compressed'
            self._sub = self.create_subscription(CompressedImage, topic,
                                                 self._on_image, qos)
        else:
            topic = self._image_topic
            self._sub = self.create_subscription(Image, topic, self._on_image, qos)
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

    def _worker_loop(self):
        """Continuously run inference on the latest available frame."""
        while not self._shutdown.is_set():
            if not self._frame_event.wait(timeout=0.2):
                continue
            self._frame_event.clear()
            while not self._shutdown.is_set():
                with self._frame_lock:
                    msg = self._latest_msg
                    self._latest_msg = None
                    drops = self._pending_drops
                    self._pending_drops = 0
                if msg is None:
                    break
                if drops:
                    with self._stats_lock:
                        self._drops += drops
                    if self._warn_on_drop:
                        self.get_logger().warn(f'Dropped {drops} stale frame(s)')
                self._process(msg)

    # ---------------------------------------------------------------- pipeline

    def _process(self, msg):
        """Decode, infer and publish results for a single image message."""
        started = time.monotonic()
        try:
            bgr = (image_utils.compressed_to_bgr(msg) if self._compressed
                   else image_utils.ros_image_to_bgr(msg))
        except Exception as exc:
            self._record_error(f'image decode failed: {exc}')
            return

        try:
            features, transform, _ = image_utils.preprocess(
                bgr, self._model.input_width, self._model.input_height,
                self._resize_mode, self._model.grayscale)
            envelope = self._runner.classify(features)
        except Exception as exc:
            self._record_error(f'inference failed: {exc}')
            return

        result = envelope.get('result', {}) if isinstance(envelope, dict) else {}
        timing = envelope.get('timing', {}) if isinstance(envelope, dict) else {}

        if not self._logged_result and isinstance(result, dict):
            self._logged_result = True
            self.get_logger().info(f'First inference result keys: {sorted(result)}')

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self._frame_id_override or msg.header.frame_id

        boxes = None
        if self._detections_pub is not None:
            boxes = conversions.extract_boxes(result, self._threshold, transform)
            self._detections_pub.publish(conversions.make_detection_array(boxes, header))

        if self._classification_pub is not None:
            items = conversions.extract_classification(result, self._threshold)
            self._classification_pub.publish(conversions.make_classification(items, header))

        anomaly = None
        if self._anomaly_pub is not None:
            anomaly = conversions.extract_anomaly(result)
            if anomaly is not None:
                self._anomaly_pub.publish(Float32(data=float(anomaly[0])))

        if self._debug_pub is not None:
            self._publish_debug_image(bgr, boxes, result, header)

        latency_ms = (time.monotonic() - started) * 1000.0
        with self._stats_lock:
            self._frames += 1
            self._last_latency_ms = latency_ms
            self._ei_timing = timing if isinstance(timing, dict) else {}
            self._last_anomaly = anomaly
            self._last_error = ''

    def _publish_debug_image(self, bgr, boxes, result, header):
        """Render and publish an annotated copy of the input frame."""
        overlay = [(b.label, b.score, b.x, b.y, b.w, b.h) for b in (boxes or [])]
        # Visual anomaly boxes are all the same class, so skip their labels.
        draw_labels = self._overlay_labels and not self._model.has_visual_anomaly
        annotated = image_utils.draw_detections(bgr, overlay, draw_labels)
        if not overlay and self._overlay_labels:
            items = conversions.extract_classification(result, None)
            if items:
                annotated = image_utils.draw_caption(
                    annotated, f'{items[0][0]} {items[0][1]:.2f}')
        self._debug_pub.publish(image_utils.bgr_to_ros_image(annotated, header))

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
            latency = self._last_latency_ms
            timing = dict(self._ei_timing)
            anomaly = self._last_anomaly
            last_error = self._last_error
            window = frames - self._diag_last_frames
            elapsed = now - self._diag_last_time
            self._diag_last_frames = frames
            self._diag_last_time = now
        fps = (window / elapsed) if elapsed > 0 else 0.0

        status = DiagnosticStatus()
        status.name = f'{self.get_name()}: inference'
        status.hardware_id = self._model.name or 'edge_impulse'
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
            KeyValue(key='model', value=f'{self._model.owner}/{self._model.name}'),
            KeyValue(key='model_type', value=self._model.model_type),
            KeyValue(key='frames', value=str(frames)),
            KeyValue(key='dropped_frames', value=str(drops)),
            KeyValue(key='fps', value=f'{fps:.2f}'),
            KeyValue(key='latency_ms', value=f'{latency:.1f}'),
            KeyValue(key='dsp_ms', value=str(timing.get('dsp', ''))),
            KeyValue(key='classification_ms', value=str(timing.get('classification', ''))),
            KeyValue(key='anomaly_ms', value=str(timing.get('anomaly', ''))),
        ]
        if anomaly is not None:
            status.values.append(KeyValue(key='anomaly_max', value=f'{anomaly[0]:.3f}'))
            status.values.append(KeyValue(key='anomaly_mean', value=f'{anomaly[1]:.3f}'))

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status.append(status)
        self._diag_pub.publish(array)

    # ---------------------------------------------------------------- shutdown

    def destroy_node(self):
        """Stop the worker thread and the model process, then tear down."""
        self._shutdown.set()
        self._frame_event.set()
        worker = getattr(self, '_worker', None)
        if worker is not None and worker.is_alive():
            worker.join(timeout=3.0)
        runner = getattr(self, '_runner', None)
        if runner is not None:
            runner.stop()
        super().destroy_node()


def main(args=None):
    """Console-script entry point for the ``edgeimpulse_detector`` executable."""
    rclpy.init(args=args)
    node = None
    try:
        node = EdgeImpulseDetector()
    except Exception as exc:  # noqa: BLE001 - surface any startup failure cleanly
        print(f'[edgeimpulse_detector] failed to start: {exc}', file=sys.stderr)
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
