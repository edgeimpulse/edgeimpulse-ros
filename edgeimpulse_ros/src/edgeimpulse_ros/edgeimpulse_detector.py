import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import threading
import json
import sys
import os
import time
import signal

try:
    from edge_impulse_linux.image import ImageImpulseRunner
except Exception:
    ImageImpulseRunner = None
try:
    import cv2
except Exception:
    cv2 = None


class EdgeImpulseDetector(Node):
    def __init__(self):
        super().__init__('edgeimpulse_detector')

        # Parameters
        self.declare_parameter('sdk_script', '')
        self.declare_parameter('model_path', '')
        self.declare_parameter('camera', 0)

        self.sdk_script = self.get_parameter('sdk_script').get_parameter_value().string_value
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.camera = self.get_parameter('camera').get_parameter_value().integer_value

        if not self.sdk_script:
            self.get_logger().error('Parameter "sdk_script" not set. Path to classify.py required.')
            raise RuntimeError('sdk_script parameter not set')
        if not os.path.exists(self.sdk_script):
            self.get_logger().error(f'sdk_script not found: {self.sdk_script}')
            raise RuntimeError('sdk_script not found')
        if not self.model_path:
            self.get_logger().error('Parameter "model_path" not set. Provide .eim model path.')
            raise RuntimeError('model_path parameter not set')

        self.pub = self.create_publisher(String, 'edgeimpulse/detections', 10)

        # Validate imports
        if ImageImpulseRunner is None:
            self.get_logger().error('edge_impulse_linux SDK not available. Install or add to PYTHONPATH.')
            raise RuntimeError('edge_impulse_linux SDK not available')
        if cv2 is None:
            self.get_logger().error('OpenCV (cv2) not available. Install via pip install opencv-python')
            raise RuntimeError('cv2 not available')

        # Start runner thread
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._runner_thread)
        self._thread.daemon = True
        self._thread.start()

        self._runner = None

    def _runner_thread(self):
        try:
            with ImageImpulseRunner(self.model_path) as runner:
                self._runner = runner
                model_info = runner.init()
                self.get_logger().info('Loaded runner for "' + model_info['project']['owner'] + ' / ' + model_info['project']['name'] + '"')

                video_id = int(self.camera)
                # Try opening camera to ensure it's valid
                cam = cv2.VideoCapture(video_id)
                if not cam.isOpened():
                    cam.release()
                    raise RuntimeError(f'Cannot open camera {video_id}')
                cam.release()

                # Iterate classifier results
                for res, img in runner.classifier(video_id):
                    if self._stop_event.is_set():
                        break
                    if not res or 'result' not in res:
                        continue
                    result = res['result']
                    if 'bounding_boxes' in result:
                        boxes = []
                        for bb in result['bounding_boxes']:
                            boxes.append({
                                'label': bb.get('label'),
                                'score': float(bb.get('value', 0)),
                                'x': int(bb.get('x', 0)),
                                'y': int(bb.get('y', 0)),
                                'w': int(bb.get('width', bb.get('w', 0))),
                                'h': int(bb.get('height', bb.get('h', 0))),
                            })
                        self._publish_boxes(boxes)

        except Exception as e:
            self.get_logger().error('Runner error: ' + str(e))
        finally:
            self.get_logger().info('Runner thread exiting')

    def _publish_boxes(self, boxes):
        msg = String()
        msg.data = json.dumps({'boxes': boxes})
        self.pub.publish(msg)
        self.get_logger().info(f'Published {len(boxes)} boxes')

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
