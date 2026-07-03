"""Unit tests for :mod:`edgeimpulse_ros.conversions` against real vision_msgs."""

from edgeimpulse_ros import conversions
from std_msgs.msg import Header


class Identity:
    """A no-op transform that leaves box coordinates unchanged."""

    def map_box(self, x, y, w, h):
        """Return the box unchanged."""
        return x, y, w, h


def _header():
    """Build a simple header for tests."""
    header = Header()
    header.frame_id = 'camera_optical_frame'
    return header


def test_extract_boxes_filters_and_maps():
    """Boxes below threshold are dropped and the transform is applied."""
    result = {
        'bounding_boxes': [
            {'label': 'cat', 'value': 0.9, 'x': 1, 'y': 2, 'width': 3, 'height': 4},
            {'label': 'dog', 'value': 0.1, 'x': 0, 'y': 0, 'width': 1, 'height': 1},
        ]
    }
    boxes = conversions.extract_boxes(result, 0.5, Identity())
    assert len(boxes) == 1
    assert boxes[0].label == 'cat'
    assert (boxes[0].x, boxes[0].y, boxes[0].w, boxes[0].h) == (1, 2, 3, 4)


def test_extract_boxes_includes_visual_anomaly_grid():
    """Visual anomaly grid cells are treated as detection boxes."""
    result = {'visual_anomaly_grid': [
        {'label': 'anomaly', 'value': 0.8, 'x': 5, 'y': 5, 'width': 2, 'height': 2}]}
    boxes = conversions.extract_boxes(result, None, None)
    assert len(boxes) == 1
    assert boxes[0].label == 'anomaly'


def test_make_detection_array_sets_class_id_and_center():
    """The detection message carries the label, score, centre and size."""
    boxes = [conversions.Box('cat', 0.9, 10, 20, 4, 6)]
    msg = conversions.make_detection_array(boxes, _header())
    assert len(msg.detections) == 1
    det = msg.detections[0]
    hypothesis = det.results[0].hypothesis
    assert hypothesis.class_id == 'cat'
    assert abs(hypothesis.score - 0.9) < 1e-6
    assert abs(det.bbox.size_x - 4) < 1e-6
    center = det.bbox.center
    cx = center.position.x if hasattr(center, 'position') else center.x
    assert abs(cx - 12) < 1e-6  # 10 + 4 / 2


def test_extract_classification_sorted_and_thresholded():
    """Classification results are filtered by threshold and sorted descending."""
    result = {'classification': {'a': 0.2, 'b': 0.7, 'c': 0.05}}
    items = conversions.extract_classification(result, 0.1)
    assert [label for label, _ in items] == ['b', 'a']


def test_make_classification_populates_results():
    """Each classification entry becomes an ObjectHypothesis."""
    msg = conversions.make_classification([('b', 0.7), ('a', 0.2)], _header())
    assert [r.class_id for r in msg.results] == ['b', 'a']


def test_extract_anomaly_visual_and_scalar():
    """Both visual and scalar anomaly shapes are recognised."""
    assert conversions.extract_anomaly(
        {'visual_anomaly_max': 0.9, 'visual_anomaly_mean': 0.3}) == (0.9, 0.3)
    assert conversions.extract_anomaly({'anomaly': 0.4}) == (0.4, 0.4)
    assert conversions.extract_anomaly({'classification': {}}) is None


def test_make_vision_info_fields():
    """Ensure VisionInfo carries the method and database location."""
    msg = conversions.make_vision_info('edge_impulse:classification',
                                       '/node/class_labels', _header())
    assert msg.method == 'edge_impulse:classification'
    assert msg.database_location == '/node/class_labels'


def test_make_label_info_maps_indices():
    """Ensure LabelInfo assigns sequential class ids to labels."""
    msg = conversions.make_label_info(['cat', 'dog'], 0.5, _header())
    assert msg is not None
    assert [(c.class_id, c.class_name) for c in msg.class_map] == [(0, 'cat'), (1, 'dog')]
