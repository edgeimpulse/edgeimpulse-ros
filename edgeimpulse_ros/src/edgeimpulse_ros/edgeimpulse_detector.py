import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import subprocess
import threading
import re
import json
import shlex
import sys
import os


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

        # Start the subprocess that runs the Edge Impulse example classifier
        cmd = [sys.executable, self.sdk_script, self.model_path, str(self.camera)]
        self.get_logger().info('Starting SDK subprocess: ' + ' '.join(shlex.quote(c) for c in cmd))
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=1, text=True)

        # Start reader thread
        self._thread = threading.Thread(target=self._reader)
        self._thread.daemon = True
        self._thread.start()

        self._buffer = []

        # Patterns
        self.re_found = re.compile(r'Found (\d+) bounding boxes')
        self.re_box = re.compile(r"(?P<label>[^\(]+) \((?P<score>\d+\.\d+)\): x=(?P<x>-?\d+) y=(?P<y>-?\d+) w=(?P<w>-?\d+) h=(?P<h>-?\d+)")

    def _reader(self):
        # Read lines and parse groups of detections
        current_boxes = []
        for raw in self.proc.stdout:
            line = raw.strip()
            if not line:
                continue
            self.get_logger().debug('SDK: ' + line)
            m = self.re_found.search(line)
            if m:
                # new group - if we had boxes, publish them
                if current_boxes:
                    self._publish_boxes(current_boxes)
                    current_boxes = []
                continue
            m2 = self.re_box.search(line)
            if m2:
                box = {
                    'label': m2.group('label').strip(),
                    'score': float(m2.group('score')),
                    'x': int(m2.group('x')),
                    'y': int(m2.group('y')),
                    'w': int(m2.group('w')),
                    'h': int(m2.group('h')),
                }
                current_boxes.append(box)

        # subprocess ended, publish any remaining boxes
        if current_boxes:
            self._publish_boxes(current_boxes)

    def _publish_boxes(self, boxes):
        msg = String()
        msg.data = json.dumps({'boxes': boxes})
        self.pub.publish(msg)
        self.get_logger().info(f'Published {len(boxes)} boxes')

    def destroy_node(self):
        # terminate subprocess cleanly
        try:
            if self.proc and self.proc.poll() is None:
                self.proc.terminate()
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
