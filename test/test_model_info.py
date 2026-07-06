"""Unit tests for :mod:`edgeimpulse_ros.model_runner` metadata parsing."""

from edgeimpulse_ros import model_runner


def _sample(model_type, has_anomaly=0, channels=3):
    """Return a minimal Edge Impulse ``init()`` dictionary."""
    return {
        'project': {'owner': 'acme', 'name': 'widgets'},
        'model_parameters': {
            'model_type': model_type,
            'image_input_width': 96,
            'image_input_height': 96,
            'image_channel_count': channels,
            'labels': ['a', 'b'],
            'has_anomaly': has_anomaly,
            'image_resize_mode': 'squash',
            'threshold': 0.6,
        },
    }


def test_parse_object_detection():
    """Object detection models expose a detection output kind."""
    info = model_runner.parse_model_info(_sample('object_detection'))
    assert info.owner == 'acme'
    assert info.name == 'widgets'
    assert info.output_kinds == {model_runner.KIND_DETECTION}
    assert info.resize_mode == 'squash'
    assert info.default_threshold == 0.6
    assert info.is_image_model


def test_parse_classification_with_anomaly():
    """Classification + anomaly exposes both output kinds."""
    info = model_runner.parse_model_info(_sample('classification', has_anomaly=1))
    assert info.output_kinds == {
        model_runner.KIND_CLASSIFICATION, model_runner.KIND_ANOMALY}


def test_parse_visual_anomaly():
    """A visual-anomaly (FOMO-AD) model emits detection + anomaly kinds."""
    info = model_runner.parse_model_info(_sample('classification', has_anomaly=4))
    assert info.has_visual_anomaly
    assert info.output_kinds == {
        model_runner.KIND_DETECTION, model_runner.KIND_ANOMALY}


def test_parse_grayscale_flag():
    """A single-channel model reports grayscale."""
    info = model_runner.parse_model_info(_sample('classification', channels=1))
    assert info.grayscale


def test_parse_defaults_on_empty():
    """An empty dict parses to safe defaults without raising."""
    info = model_runner.parse_model_info({})
    assert info.output_kinds == set()
    assert not info.is_image_model
