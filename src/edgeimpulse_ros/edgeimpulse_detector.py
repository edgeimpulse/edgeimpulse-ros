import os
import threading
import sys
import site
from typing import Optional

import rclpy
from rclpy.node import Node


class EdgeImpulseDetector(Node):
    def __init__(self):
        super().__init__('edgeimpulse_detector')

        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('camera', 0)
        self.declare_parameter('score_threshold', 0.5)
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('detections_topic', 'edgeimpulse/detections')

        self._model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self._camera = int(self.get_parameter('camera').value)
        self._score_threshold = float(self.get_parameter('score_threshold').value)
        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self._detections_topic = self.get_parameter('detections_topic').get_parameter_value().string_value

        if not self._model_path:
            raise RuntimeError('Parameter `model_path` is required (path to .eim file)')

        self._model_path = os.path.expanduser(self._model_path)

        # ROS launch environments sometimes disable user site-packages (e.g. PYTHONNOUSERSITE=1).
        # If the EI SDK was installed with `pip --user`, it lives under the user site directory.
        try:
            user_site = site.getusersitepackages()
            if user_site and user_site not in sys.path:
                site.addsitedir(user_site)
        except Exception:
            user_site = ''

        try:
            import edge_impulse_linux  # noqa: F401,WPS433
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                'Python module `edge_impulse_linux` not found for this interpreter:\n'
                f'  sys.executable={sys.executable}\n\n'
                f'  user_site={user_site}\n\n'
                'Install the Edge Impulse Linux Python SDK into the same Python environment, e.g.:\n'
                f'  {sys.executable} -m pip install --user git+https://github.com/edgeimpulse/linux-sdk-python.git\n\n'
                'If you are using a virtualenv, activate it BEFORE `colcon build`, then rebuild.'
            ) from exc

        try:
            from vision_msgs.msg import Detection2DArray  # noqa: WPS433
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                'Python module `vision_msgs` not found. Install it via your ROS distro packages, e.g.\n'
                '  sudo apt update && sudo apt install ros-$ROS_DISTRO-vision-msgs\n'
                'Then re-source your environment and try again.'
            ) from exc

        self._pub = self.create_publisher(Detection2DArray, self._detections_topic, 10)

        # Start runner thread
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._runner_thread)
        self._thread.daemon = True
        self._thread.start()

        self._runner: Optional[object] = None

    def _runner_thread(self):
        try:
            from edge_impulse_linux.image import ImageImpulseRunner  # noqa: WPS433

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
            self.get_logger().error('Runner error: ' + str(e))
        finally:
            self.get_logger().info('Runner thread exiting')

    def _publish_result(self, res: dict):
        from vision_msgs.msg import (  # noqa: WPS433
            BoundingBox2D,
            Detection2D,
            Detection2DArray,
            ObjectHypothesisWithPose,
        )
        from vision_msgs.msg import ObjectHypothesis  # noqa: WPS433
        from geometry_msgs.msg import Pose2D  # noqa: WPS433

        if not isinstance(res, dict) or 'result' not in res:
            return

        result = res.get('result', {})
        bbs = result.get('bounding_boxes', None)
        if bbs is None:
            return

        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        for bb in bbs:
            try:
                label = bb.get('label', '')
                score = float(bb.get('value', 0.0))
                if score < self._score_threshold:
                    continue

                x = int(bb.get('x', 0))
                y = int(bb.get('y', 0))
                w = int(bb.get('width', bb.get('w', 0)))
                h = int(bb.get('height', bb.get('h', 0)))

                det = Detection2D()
                det.header = msg.header

                bbox = BoundingBox2D()
                center = Pose2D()
                center.x = float(x) + float(w) / 2.0
                center.y = float(y) + float(h) / 2.0
                center.theta = 0.0
                bbox.center = center
                bbox.size_x = float(w)
                bbox.size_y = float(h)
                det.bbox = bbox

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis = ObjectHypothesis()
                hyp.hypothesis.class_id = str(label)
                hyp.hypothesis.score = float(score)
                det.results.append(hyp)

                msg.detections.append(det)

            except Exception:
                continue

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
        rclpy.shutdown()
