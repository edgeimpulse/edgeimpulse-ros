"""Unit tests for :mod:`edgeimpulse_ros.image_utils` (no ROS graph required)."""

from edgeimpulse_ros import image_utils
import numpy as np


class FakeImage:
    """Minimal stand-in for ``sensor_msgs/Image`` used by the decoder tests."""

    def __init__(self, encoding, height, width, data, step=0):
        """Store the fields ``ros_image_to_bgr`` reads."""
        self.encoding = encoding
        self.height = height
        self.width = width
        self.data = data
        self.step = step


def test_ros_image_to_bgr_bgr8_roundtrip():
    """A bgr8 image is returned unchanged."""
    img = np.arange(2 * 2 * 3, dtype=np.uint8).reshape(2, 2, 3)
    msg = FakeImage('bgr8', 2, 2, img.tobytes(), step=2 * 3)
    out = image_utils.ros_image_to_bgr(msg)
    assert out.shape == (2, 2, 3)
    assert np.array_equal(out, img)


def test_ros_image_to_bgr_rgb8_swaps_channels():
    """An rgb8 image comes back with channels swapped to BGR."""
    rgb = np.array([[[10, 20, 30]]], dtype=np.uint8)
    msg = FakeImage('rgb8', 1, 1, rgb.tobytes(), step=3)
    out = image_utils.ros_image_to_bgr(msg)
    assert list(out[0, 0]) == [30, 20, 10]


def test_ros_image_to_bgr_mono8_replicates():
    """A mono8 image is expanded to three equal channels."""
    mono = np.array([[7, 9]], dtype=np.uint8)
    msg = FakeImage('mono8', 1, 2, mono.tobytes(), step=2)
    out = image_utils.ros_image_to_bgr(msg)
    assert out.shape == (1, 2, 3)
    assert list(out[0, 0]) == [7, 7, 7]


def test_ros_image_to_bgr_honours_row_padding():
    """A bgr8 image with a padded step is de-padded correctly."""
    row = np.array([1, 2, 3, 4, 5, 6, 0, 0], dtype=np.uint8)  # 2 px + 2 pad bytes
    msg = FakeImage('bgr8', 1, 2, row.tobytes(), step=8)
    out = image_utils.ros_image_to_bgr(msg)
    assert out.shape == (1, 2, 3)
    assert list(out[0, 1]) == [4, 5, 6]


def test_ros_image_to_bgr_nv12_shape():
    """An nv12 buffer decodes to a full-size BGR image."""
    height, width = 4, 4
    data = np.full(height * width * 3 // 2, 128, dtype=np.uint8).tobytes()
    msg = FakeImage('nv12', height, width, data, step=width)
    out = image_utils.ros_image_to_bgr(msg)
    assert out.shape == (height, width, 3)


def test_ros_image_to_bgr_rejects_unknown_encoding():
    """An unsupported encoding raises ``ValueError``."""
    msg = FakeImage('bayer_rggb8', 1, 1, b'\x00', step=1)
    raised = False
    try:
        image_utils.ros_image_to_bgr(msg)
    except ValueError:
        raised = True
    assert raised


def test_pack_features_rgb():
    """Colour pixels pack as ``(r << 16) | (g << 8) | b``."""
    bgr = np.array([[[3, 2, 1]]], dtype=np.uint8)  # b=3, g=2, r=1
    feats = image_utils.pack_features(bgr, grayscale=False)
    assert feats == [(1 << 16) | (2 << 8) | 3]


def test_pack_features_grayscale():
    """Grayscale pixels replicate the value across channels."""
    bgr = np.array([[[5, 5, 5]]], dtype=np.uint8)
    feats = image_utils.pack_features(bgr, grayscale=True)
    assert feats == [(5 << 16) | (5 << 8) | 5]


def test_preprocess_squash_transform():
    """Squash resize maps the full model box back to the full source."""
    src = np.zeros((2, 4, 3), dtype=np.uint8)
    feats, transform, model = image_utils.preprocess(src, 2, 2, 'squash', False)
    assert len(feats) == 4
    assert model.shape == (2, 2, 3)
    x, y, w, h = transform.map_box(0, 0, 2, 2)
    assert (round(x), round(y), round(w), round(h)) == (0, 0, 4, 2)


def test_preprocess_fit_shortest_centered():
    """Fit-shortest centre-crops, so a model box maps to a centred source box."""
    src = np.zeros((2, 4, 3), dtype=np.uint8)
    _, transform, model = image_utils.preprocess(src, 2, 2, 'fit-shortest', False)
    assert model.shape == (2, 2, 3)
    x, y, w, h = transform.map_box(0, 0, 2, 2)
    assert (round(x), round(y), round(w), round(h)) == (1, 0, 2, 2)


def test_preprocess_fit_longest_letterbox():
    """Fit-longest pads, so the non-padded model region maps to full source."""
    src = np.zeros((2, 4, 3), dtype=np.uint8)
    _, transform, model = image_utils.preprocess(src, 2, 2, 'fit-longest', False)
    assert model.shape == (2, 2, 3)
    x, y, w, h = transform.map_box(0, 0, 2, 1)
    assert (round(x), round(y), round(w), round(h)) == (0, 0, 4, 2)
