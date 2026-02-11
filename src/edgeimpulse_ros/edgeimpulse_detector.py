import os
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from edge_impulse_linux.image import ImageImpulseRunner
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)


class EdgeImpulseDetector(Node):
    def __init__(self):
        super().__init__('edgeimpulse_detector')

        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('camera', 0)
        self.declare_parameter('score_threshold', 0.5)
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('detections_topic', 'edgeimpulse/detections')
        self.declare_parameter('publish_empty', False)
        self.declare_parameter('log_detections', True)
        self.declare_parameter('log_raw_bounding_boxes', True)
        self.declare_parameter('status_period_sec', 5.0)

        self._model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self._camera = int(self.get_parameter('camera').value)
        self._score_threshold = float(self.get_parameter('score_threshold').value)
        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self._detections_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        self._publish_empty = bool(self.get_parameter('publish_empty').value)
        self._log_detections = bool(self.get_parameter('log_detections').value)
        self._log_raw_bounding_boxes = bool(self.get_parameter('log_raw_bounding_boxes').value)
        self._status_period_sec = float(self.get_parameter('status_period_sec').value)

        if not self._model_path:
            raise RuntimeError('Parameter `model_path` is required (path to .eim file)')

        self._model_path = os.path.expanduser(self._model_path)

        self._pub = self.create_publisher(Detection2DArray, self._detections_topic, 10)

        self._frames_total = 0
        self._frames_with_detections = 0
        self._last_status_time = time.monotonic()
        self._bb_error_count = 0
        self._last_bb_error_log_time = 0.0

        # Start runner thread
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._runner_thread)
        self._thread.daemon = True
        self._thread.start()

        self._runner: Optional[object] = None

    def _runner_thread(self):
        try:
            with ImageImpulseRunner(self._model_path) as runner:
                self._runner = runner
                model_info = runner.init()
                project = model_info.get('project', {}) if isinstance(model_info, dict) else {}
                self.get_logger().info(
                    f'Loaded EI model: {project.get("owner", "?")} / {project.get("name", "?")}. '
                    f'camera={self._camera} score_threshold={self._score_threshold}'
                )

                for res, _img in runner.classifier(self._camera):
                    if self._stop_event.is_set():
                        break
                    self._publish_result(res)

        except Exception as e:
            # The EI runner can throw exceptions while stopping (e.g. broken pipe / connection reset).
            if self._stop_event.is_set():
                self.get_logger().info('Runner stopped')
            else:
                self.get_logger().error('Runner error: ' + str(e))
        finally:
            self.get_logger().info('Runner thread exiting')

    def _publish_result(self, res: dict):
        if not isinstance(res, dict) or 'result' not in res:
            return

        result = res.get('result', {})
        bbs = result.get('bounding_boxes', None)
        if bbs is None:
            return

        self._frames_total += 1

        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        raw_bb_count = 0
        published_bb_count = 0

        for bb in bbs:
            raw_bb_count += 1
            try:
                label = bb.get('label', '')
                score = float(bb.get('value', 0.0))

                x = int(bb.get('x', 0))
                y = int(bb.get('y', 0))
                w = int(bb.get('width', bb.get('w', 0)))
                h = int(bb.get('height', bb.get('h', 0)))

                if self._log_raw_bounding_boxes:
                    self.get_logger().info(
                        f'RAW {label} ({score:.2f}): x={x} y={y} w={w} h={h}'
                    )

                if score < self._score_threshold:
                    continue

                det = Detection2D()
                det.header = msg.header

                bbox = BoundingBox2D()
                # `vision_msgs/BoundingBox2D.center` differs across releases:
                # - some use `geometry_msgs/Pose2D` (x, y, theta)
                # - some use `geometry_msgs/Pose` (position.x, position.y)
                cx = float(x) + float(w) / 2.0
                cy = float(y) + float(h) / 2.0
                center = bbox.center
                if hasattr(center, 'x'):
                    center.x = cx
                    center.y = cy
                    if hasattr(center, 'theta'):
                        center.theta = 0.0
                elif hasattr(center, 'position'):
                    center.position.x = cx
                    center.position.y = cy
                    if hasattr(center.position, 'z'):
                        center.position.z = 0.0
                    if hasattr(center, 'orientation') and hasattr(center.orientation, 'w'):
                        center.orientation.w = 1.0
                bbox.size_x = float(w)
                bbox.size_y = float(h)
                det.bbox = bbox

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis = ObjectHypothesis()
                # vision_msgs differs across ROS distros: some have `class_id` (string), others have `id` (int).
                if hasattr(hyp.hypothesis, 'class_id'):
                    hyp.hypothesis.class_id = str(label)
                elif hasattr(hyp.hypothesis, 'id'):
                    hyp.hypothesis.id = 0
                hyp.hypothesis.score = float(score)
                det.results.append(hyp)

                msg.detections.append(det)
                published_bb_count += 1

                if self._log_detections:
                    self.get_logger().info(
                        f'{label} ({score:.2f}): x={x} y={y} w={w} h={h}'
                    )

            except Exception as exc:
                self._bb_error_count += 1
                now = time.monotonic()
                # Throttle to avoid log spam.
                if (now - self._last_bb_error_log_time) > 2.0:
                    self.get_logger().warning(
                        f'Failed to convert/publish a bounding box (count={self._bb_error_count}): {exc}; bb={bb}'
                    )
                    self._last_bb_error_log_time = now
                continue

        if published_bb_count > 0:
            self._frames_with_detections += 1

        now = time.monotonic()
        if self._status_period_sec > 0 and (now - self._last_status_time) >= self._status_period_sec:
            self.get_logger().info(
                f'Frames={self._frames_total} frames_with_detections={self._frames_with_detections} '
                f'last_raw_boxes={raw_bb_count} last_published_boxes={published_bb_count}'
            )
            self._last_status_time = now

        if published_bb_count > 0 or self._publish_empty:
            self._pub.publish(msg)

    def destroy_node(self):
        # signal runner thread to stop
        try:
            self._stop_event.set()
            if self._runner:
                try:
                    self._runner.stop()
                except Exception:
                    pass
            if self._thread.is_alive():
                self._thread.join(timeout=2.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = EdgeImpulseDetector()
    except Exception as e:
        print('Failed to start node:', e)
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Avoid crashing if shutdown already happened (e.g. launch signal handling).
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
