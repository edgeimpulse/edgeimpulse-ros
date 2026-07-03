"""
Pure converters from Edge Impulse result dictionaries to ``vision_msgs``.

Everything here is side-effect free and independent of ``rclpy`` and OpenCV so
it can be unit tested against the real message types with nothing more than a
sourced ROS installation. Small differences between ``vision_msgs`` releases
(for example ``ObjectHypothesis.class_id`` vs ``id``, or the ``BoundingBox2D``
centre type) are handled here in one place via duck typing.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    VisionInfo,
)

try:  # Humble+ / Jazzy
    from vision_msgs.msg import Classification as ClassificationMsg
except ImportError:  # pragma: no cover - older distros
    from vision_msgs.msg import Classification2D as ClassificationMsg

try:  # Humble+ / Jazzy
    from vision_msgs.msg import LabelInfo, VisionClass
    _HAS_LABEL_INFO = True
except ImportError:  # pragma: no cover - older distros
    _HAS_LABEL_INFO = False

__all__ = [
    'Box',
    'extract_boxes',
    'extract_classification',
    'extract_anomaly',
    'make_detection_array',
    'make_classification',
    'make_vision_info',
    'make_label_info',
    'has_label_info',
]


@dataclass
class Box:
    """A detection box in source-image pixel coordinates."""

    label: str
    score: float
    x: float
    y: float
    w: float
    h: float


def _set_hypothesis(hypothesis, label: str, score: float) -> None:
    """Populate an ``ObjectHypothesis`` across vision_msgs releases."""
    if hasattr(hypothesis, 'class_id'):  # modern: string identifier
        hypothesis.class_id = str(label)
    elif hasattr(hypothesis, 'id'):  # legacy: integer identifier
        hypothesis.id = 0
    hypothesis.score = float(score)


def _set_center(bbox: BoundingBox2D, cx: float, cy: float) -> None:
    """Set a ``BoundingBox2D`` centre across the two Pose2D layouts."""
    center = bbox.center
    if hasattr(center, 'position'):  # vision_msgs/Pose2D (Point2D position)
        center.position.x = float(cx)
        center.position.y = float(cy)
    else:  # geometry_msgs/Pose2D (x, y directly)
        center.x = float(cx)
        center.y = float(cy)


def extract_boxes(result: dict, threshold: Optional[float], transform=None) -> List[Box]:
    """
    Extract detection/visual-anomaly boxes, mapped to source coordinates.

    ``transform`` may be any object exposing ``map_box(x, y, w, h)``; when
    ``None`` the raw model-space coordinates are kept. ``threshold`` filters by
    confidence when not ``None``.
    """
    raw: List[dict] = []
    if isinstance(result.get('bounding_boxes'), list):
        raw.extend(result['bounding_boxes'])
    if isinstance(result.get('visual_anomaly_grid'), list):
        raw.extend(result['visual_anomaly_grid'])

    boxes: List[Box] = []
    for bb in raw:
        score = float(bb.get('value', 0.0) or 0.0)
        if threshold is not None and score < threshold:
            continue
        x = float(bb.get('x', 0.0) or 0.0)
        y = float(bb.get('y', 0.0) or 0.0)
        w = float(bb.get('width', bb.get('w', 0.0)) or 0.0)
        h = float(bb.get('height', bb.get('h', 0.0)) or 0.0)
        if transform is not None:
            x, y, w, h = transform.map_box(x, y, w, h)
        boxes.append(Box(str(bb.get('label', '')), score, x, y, w, h))
    return boxes


def extract_classification(result: dict,
                           threshold: Optional[float]) -> List[Tuple[str, float]]:
    """Return ``(label, score)`` pairs sorted by descending score."""
    classification = result.get('classification', {})
    items: List[Tuple[str, float]] = []
    if isinstance(classification, dict):
        for label, score in classification.items():
            value = float(score)
            if threshold is not None and value < threshold:
                continue
            items.append((str(label), value))
    items.sort(key=lambda kv: kv[1], reverse=True)
    return items


def extract_anomaly(result: dict) -> Optional[Tuple[float, float]]:
    """
    Return ``(max, mean)`` anomaly scores, or ``None`` when not present.

    Handles both scalar anomaly (K-means/GMM) and visual anomaly (FOMO-AD).
    """
    if 'visual_anomaly_max' in result or 'visual_anomaly_mean' in result:
        max_score = float(result.get('visual_anomaly_max', 0.0) or 0.0)
        mean_score = float(result.get('visual_anomaly_mean', 0.0) or 0.0)
        return max_score, mean_score
    if 'anomaly' in result:
        score = float(result.get('anomaly', 0.0) or 0.0)
        return score, score
    return None


def make_detection_array(boxes: List[Box], header) -> Detection2DArray:
    """Build a ``Detection2DArray`` from source-space boxes."""
    array = Detection2DArray()
    array.header = header
    for box in boxes:
        det = Detection2D()
        det.header = header

        bbox = BoundingBox2D()
        _set_center(bbox, box.x + box.w / 2.0, box.y + box.h / 2.0)
        bbox.size_x = float(box.w)
        bbox.size_y = float(box.h)
        det.bbox = bbox

        hyp = ObjectHypothesisWithPose()
        _set_hypothesis(hyp.hypothesis, box.label, box.score)
        det.results.append(hyp)

        array.detections.append(det)
    return array


def make_classification(items: List[Tuple[str, float]], header) -> ClassificationMsg:
    """Build a ``vision_msgs/Classification`` from ``(label, score)`` pairs."""
    msg = ClassificationMsg()
    msg.header = header
    for label, score in items:
        hyp = ObjectHypothesis()
        _set_hypothesis(hyp, label, score)
        msg.results.append(hyp)
    return msg


def make_vision_info(method: str, database_location: str, header) -> VisionInfo:
    """Build a latched ``VisionInfo`` describing the pipeline and label source."""
    msg = VisionInfo()
    msg.header = header
    msg.method = str(method)
    msg.database_location = str(database_location)
    return msg


def has_label_info() -> bool:
    """Return ``True`` when the installed ``vision_msgs`` provides ``LabelInfo``."""
    return _HAS_LABEL_INFO


def make_label_info(labels: List[str], threshold: float, header):
    """
    Build a latched ``LabelInfo`` mapping class indices to names.

    Returns ``None`` on distributions that predate ``vision_msgs/LabelInfo``.
    """
    if not _HAS_LABEL_INFO:
        return None
    msg = LabelInfo()
    msg.header = header
    for index, name in enumerate(labels):
        entry = VisionClass()
        entry.class_id = int(index)
        entry.class_name = str(name)
        msg.class_map.append(entry)
    msg.threshold = float(threshold)
    return msg
