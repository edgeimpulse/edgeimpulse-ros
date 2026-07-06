"""
Image helpers: ROS <-> OpenCV conversion and Edge Impulse preprocessing.

This module is intentionally free of any ``rclpy`` dependency so the pure image
math can be unit tested without a running ROS graph. The only functions that
touch ROS message types import them lazily.

Two problems are solved here:

* **Encoding normalisation** - convert the many ``sensor_msgs/Image`` encodings
  (including ``nv12`` / ``nv21`` / ``yuyv`` emitted by hardware ISPs such as the
  Qualcomm QRB platform) into a plain ``bgr8`` numpy array.
* **Model preprocessing** - resize/crop/pad an image into the tensor an Edge
  Impulse model expects, while remembering the exact geometric transform so
  detections can be mapped back to the *original* image coordinates.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Tuple

import cv2
import numpy as np

__all__ = [
    'Transform',
    'ros_image_to_bgr',
    'compressed_to_bgr',
    'preprocess',
    'pack_features',
    'draw_detections',
    'draw_caption',
    'bgr_to_ros_image',
]


@dataclass(frozen=True)
class Transform:
    """
    Affine map from source-image pixels to model-input pixels.

    The relationship for each axis is ``model = source * scale + offset``.
    Inverting it lets us express model-space detections back in the coordinate
    frame of the original camera image.
    """

    scale_x: float
    scale_y: float
    offset_x: float
    offset_y: float
    source_width: int
    source_height: int

    def map_box(self, x: float, y: float, w: float,
                h: float) -> Tuple[float, float, float, float]:
        """
        Map a model-space ``(x, y, w, h)`` box to source-image pixels.

        The returned box is clamped to the source image bounds.
        """
        sx = (x - self.offset_x) / self.scale_x
        sy = (y - self.offset_y) / self.scale_y
        sw = w / self.scale_x
        sh = h / self.scale_y

        x0 = max(0.0, min(sx, float(self.source_width)))
        y0 = max(0.0, min(sy, float(self.source_height)))
        x1 = max(0.0, min(sx + sw, float(self.source_width)))
        y1 = max(0.0, min(sy + sh, float(self.source_height)))
        return x0, y0, x1 - x0, y1 - y0


# Encodings we can decode without cv_bridge are handled explicitly in
# ros_image_to_bgr below.


def _as_array(data: bytes, height: int, width: int, channels: int,
              step: int) -> np.ndarray:
    """Return an ``(H, W, C)`` uint8 view, honouring row padding via ``step``."""
    arr = np.frombuffer(data, dtype=np.uint8)
    row_bytes = width * channels
    if step and step != row_bytes:
        arr = arr.reshape(height, step)[:, :row_bytes]
    return arr.reshape(height, width, channels)


def ros_image_to_bgr(msg) -> np.ndarray:
    """
    Convert a ``sensor_msgs/Image`` into a contiguous ``bgr8`` numpy array.

    Supports the common colour/mono encodings plus the packed YUV formats
    (``yuyv``/``uyvy``) and semi-planar formats (``nv12``/``nv21``) that show up
    on embedded camera pipelines and that ``cv_bridge`` cannot decode.
    """
    encoding = (msg.encoding or '').lower()
    height = int(msg.height)
    width = int(msg.width)
    step = int(getattr(msg, 'step', 0) or 0)
    data = bytes(msg.data)

    if encoding in ('bgr8', '8uc3'):
        return np.ascontiguousarray(_as_array(data, height, width, 3, step))
    if encoding == 'rgb8':
        rgb = _as_array(data, height, width, 3, step)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if encoding == 'bgra8':
        bgra = _as_array(data, height, width, 4, step)
        return cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
    if encoding == 'rgba8':
        rgba = _as_array(data, height, width, 4, step)
        return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
    if encoding in ('mono8', '8uc1'):
        mono = _as_array(data, height, width, 1, step)
        return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
    if encoding in ('mono16', '16uc1'):
        mono16 = np.frombuffer(data, dtype=np.uint16).reshape(height, width)
        mono8 = cv2.convertScaleAbs(mono16, alpha=255.0 / 65535.0)
        return cv2.cvtColor(mono8, cv2.COLOR_GRAY2BGR)
    if encoding in ('yuv422_yuy2', 'yuyv'):
        yuv = _as_array(data, height, width, 2, step)
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YUYV)
    if encoding in ('yuv422', 'uyvy'):
        # sensor_msgs historically labels UYVY as "yuv422".
        yuv = _as_array(data, height, width, 2, step)
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_UYVY)
    if encoding == 'nv12':
        yuv = np.frombuffer(data, dtype=np.uint8).reshape(height * 3 // 2, width)
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
    if encoding == 'nv21':
        yuv = np.frombuffer(data, dtype=np.uint8).reshape(height * 3 // 2, width)
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV21)

    raise ValueError(f'Unsupported image encoding: {msg.encoding!r}')


def compressed_to_bgr(msg) -> np.ndarray:
    """Decode a ``sensor_msgs/CompressedImage`` into a ``bgr8`` numpy array."""
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError('Failed to decode CompressedImage payload')
    return bgr


def _interpolation(src_pixels: int, dst_pixels: int) -> int:
    """Pick INTER_AREA when shrinking, INTER_LINEAR when enlarging."""
    return cv2.INTER_AREA if dst_pixels < src_pixels else cv2.INTER_LINEAR


def preprocess(bgr: np.ndarray, width: int, height: int,
               mode: str = 'fit-shortest',
               grayscale: bool = False) -> Tuple[List[int], Transform, np.ndarray]:
    """
    Resize ``bgr`` to the model input and return packed features.

    Returns a ``(features, transform, model_bgr)`` tuple where ``features`` is
    the flat list of packed pixels Edge Impulse expects, ``transform`` maps
    model pixels back to the source image, and ``model_bgr`` is the resized BGR
    image (useful for debugging).

    ``mode`` is one of ``squash`` (stretch), ``fit-shortest`` (cover +
    centre-crop, the Edge Impulse SDK default) or ``fit-longest`` (contain +
    letterbox pad).
    """
    in_h, in_w = bgr.shape[:2]
    if in_h == 0 or in_w == 0:
        raise ValueError('Cannot preprocess an empty image')

    interp = _interpolation(in_w * in_h, width * height)

    if mode == 'squash':
        model_bgr = cv2.resize(bgr, (width, height), interpolation=interp)
        transform = Transform(width / in_w, height / in_h, 0.0, 0.0, in_w, in_h)
    elif mode == 'fit-longest':
        scale = min(width / in_w, height / in_h)
        new_w = max(1, min(width, round(in_w * scale)))
        new_h = max(1, min(height, round(in_h * scale)))
        resized = cv2.resize(bgr, (new_w, new_h), interpolation=interp)
        canvas = np.zeros((height, width, 3), dtype=np.uint8)
        off_x = (width - new_w) // 2
        off_y = (height - new_h) // 2
        canvas[off_y:off_y + new_h, off_x:off_x + new_w] = resized
        model_bgr = canvas
        transform = Transform(scale, scale, float(off_x), float(off_y), in_w, in_h)
    else:  # 'fit-shortest' (a.k.a. cover) is the safe default.
        scale = max(width / in_w, height / in_h)
        new_w = int(math.ceil(in_w * scale))
        new_h = int(math.ceil(in_h * scale))
        resized = cv2.resize(bgr, (new_w, new_h), interpolation=interp)
        crop_x = (new_w - width) // 2
        crop_y = (new_h - height) // 2
        model_bgr = resized[crop_y:crop_y + height, crop_x:crop_x + width]
        transform = Transform(scale, scale, float(-crop_x), float(-crop_y), in_w, in_h)

    features = pack_features(model_bgr, grayscale)
    return features, transform, model_bgr


def pack_features(model_bgr: np.ndarray, grayscale: bool) -> List[int]:
    """
    Pack a BGR model image into Edge Impulse's flat pixel feature list.

    Colour pixels are encoded as ``(r << 16) | (g << 8) | b`` and grayscale
    pixels as the same value replicated across channels, matching the
    Edge Impulse Linux SDK.
    """
    if grayscale:
        gray = cv2.cvtColor(model_bgr, cv2.COLOR_BGR2GRAY).astype(np.uint32).reshape(-1)
        packed = (gray << 16) | (gray << 8) | gray
    else:
        rgb = cv2.cvtColor(model_bgr, cv2.COLOR_BGR2RGB).astype(np.uint32)
        r = rgb[:, :, 0].reshape(-1)
        g = rgb[:, :, 1].reshape(-1)
        b = rgb[:, :, 2].reshape(-1)
        packed = (r << 16) | (g << 8) | b
    return packed.tolist()


def draw_detections(bgr: np.ndarray, detections, draw_labels: bool = True) -> np.ndarray:
    """
    Return a copy of ``bgr`` with ``detections`` drawn as labelled boxes.

    ``detections`` is an iterable of ``(label, score, x, y, w, h)`` tuples in
    source-image pixel coordinates.
    """
    out = bgr.copy()
    for label, score, x, y, w, h in detections:
        p1 = (int(round(x)), int(round(y)))
        p2 = (int(round(x + w)), int(round(y + h)))
        cv2.rectangle(out, p1, p2, (0, 0, 255), 2)
        if draw_labels:
            ty = max(0, p1[1] - 6)
            cv2.putText(out, str(label), (p1[0], ty), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 0, 255), 1, cv2.LINE_AA)
    return out


def draw_caption(bgr: np.ndarray, text: str) -> np.ndarray:
    """Return a copy of ``bgr`` with ``text`` drawn in the top-left corner."""
    out = bgr.copy()
    cv2.putText(out, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (0, 0, 255), 2, cv2.LINE_AA)
    return out


def bgr_to_ros_image(bgr: np.ndarray, header):
    """Build a ``sensor_msgs/Image`` (``bgr8``) from a numpy array."""
    from sensor_msgs.msg import Image  # local import keeps module ROS-optional

    bgr = np.ascontiguousarray(bgr)
    msg = Image()
    msg.header = header
    msg.height = int(bgr.shape[0])
    msg.width = int(bgr.shape[1])
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = int(bgr.shape[1] * 3)
    msg.data = bgr.tobytes()
    return msg
