r"""
Publish synthetic NV12 images for validating the detector's decode path.

A small test/demo helper that mimics the Qualcomm QRB camera output (``nv12``)
so you can exercise ``edgeimpulse_detector`` without any camera hardware::

    ros2 run edgeimpulse_ros nv12_test_publisher
    ros2 run edgeimpulse_ros edgeimpulse_detector --ros-args \\
        -p model_path:=/path/to/model.eim -p publish_debug_image:=true
"""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def bgr_to_nv12(bgr: np.ndarray) -> np.ndarray:
    """Convert a BGR image to a packed NV12 (Y plane + interleaved UV) buffer."""
    height, width = bgr.shape[:2]
    yuv = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV)
    y = yuv[:, :, 0]
    u = yuv[0::2, 0::2, 1]
    v = yuv[0::2, 0::2, 2]
    uv = np.empty((height // 2, width), dtype=np.uint8)
    uv[:, 0::2] = u
    uv[:, 1::2] = v
    return np.vstack((y, uv))


class Nv12Publisher(Node):
    """Publish a moving coloured rectangle encoded as ``nv12``."""

    def __init__(self):
        """Declare parameters and start the publish timer."""
        super().__init__('nv12_test_publisher')
        self._width = int(self.declare_parameter('width', 640).value)
        self._height = int(self.declare_parameter('height', 480).value)
        self._frame_id = str(self.declare_parameter('frame_id', 'camera').value)
        topic = str(self.declare_parameter('topic', 'image').value)
        rate = float(self.declare_parameter('rate', 10.0).value)

        # NV12's 2x2 chroma subsampling needs even dimensions.
        self._width -= self._width % 2
        self._height -= self._height % 2

        self._pub = self.create_publisher(Image, topic, 10)
        self._counter = 0
        self.create_timer(1.0 / max(rate, 0.1), self._tick)
        self.get_logger().info(
            f'Publishing {self._width}x{self._height} nv12 on "{topic}" '
            f'at {rate:.1f} Hz')

    def _tick(self):
        """Render one frame and publish it as an nv12 image message."""
        bgr = np.zeros((self._height, self._width, 3), dtype=np.uint8)
        box = self._width // 6
        x = (self._counter * 8) % max(1, self._width - box)
        top = self._height // 2 - box // 2
        cv2.rectangle(bgr, (x, top), (x + box, top + box), (0, 0, 255), -1)
        self._counter += 1

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.height = self._height
        msg.width = self._width
        msg.encoding = 'nv12'
        msg.is_bigendian = 0
        msg.step = self._width
        msg.data = bgr_to_nv12(bgr).tobytes()
        self._pub.publish(msg)


def main(args=None):
    """Console-script entry point for the nv12 test publisher."""
    rclpy.init(args=args)
    node = Nv12Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
