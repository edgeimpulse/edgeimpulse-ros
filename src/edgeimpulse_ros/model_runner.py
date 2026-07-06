"""
Thin, ROS-agnostic wrapper around the Edge Impulse Linux Python SDK.

The heavy lifting (spawning the ``.eim`` binary, IPC) is done by the SDK's
``ImpulseRunner``. This module adds a typed view over the model metadata so the
rest of the package never has to reach into raw dictionaries, and so the model
introspection can be unit tested without the SDK installed.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List

# Output modalities a model can produce.
KIND_DETECTION = 'detection'
KIND_CLASSIFICATION = 'classification'
KIND_ANOMALY = 'anomaly'

# Edge Impulse ``has_anomaly`` metadata: 0 = none, 1 = K-means, 2 = GMM
# (both scalar); >= 3 is visual anomaly (FOMO-AD), which also emits a grid.
_VISUAL_ANOMALY_MIN = 3


@dataclass
class ModelInfo:
    """Typed, distilled view of the Edge Impulse model metadata."""

    owner: str = ''
    name: str = ''
    model_type: str = ''
    input_width: int = 0
    input_height: int = 0
    channel_count: int = 3
    labels: List[str] = field(default_factory=list)
    has_anomaly: int = 0
    resize_mode: str = 'fit-shortest'
    default_threshold: float = 0.0
    raw: Dict = field(default_factory=dict)

    @property
    def grayscale(self) -> bool:
        """Return ``True`` when the model expects a single-channel input."""
        return self.channel_count == 1

    @property
    def is_image_model(self) -> bool:
        """Return ``True`` when the model consumes an image tensor."""
        return self.input_width > 0 and self.input_height > 0

    @property
    def has_visual_anomaly(self) -> bool:
        """Return ``True`` for visual (FOMO-AD) anomaly models with a grid."""
        return self.has_anomaly >= _VISUAL_ANOMALY_MIN

    @property
    def output_kinds(self) -> set:
        """Set of :data:`KIND_*` values this model can emit."""
        kinds = set()
        if self.has_visual_anomaly:
            # FOMO-AD emits a spatial grid (-> detections) plus a score.
            return {KIND_DETECTION, KIND_ANOMALY}
        if self.model_type in ('object_detection', 'constrained_object_detection'):
            kinds.add(KIND_DETECTION)
        elif self.model_type == 'classification':
            kinds.add(KIND_CLASSIFICATION)
        if self.has_anomaly:
            kinds.add(KIND_ANOMALY)
        return kinds


def parse_model_info(raw: dict) -> ModelInfo:
    """
    Turn the SDK's ``init()`` dictionary into a :class:`ModelInfo`.

    Missing keys fall back to sensible defaults so a slightly different SDK
    version never crashes the node at startup.
    """
    project = raw.get('project', {}) if isinstance(raw, dict) else {}
    params = raw.get('model_parameters', {}) if isinstance(raw, dict) else {}

    return ModelInfo(
        owner=str(project.get('owner', '')),
        name=str(project.get('name', '')),
        model_type=str(params.get('model_type', '')),
        input_width=int(params.get('image_input_width', 0) or 0),
        input_height=int(params.get('image_input_height', 0) or 0),
        channel_count=int(params.get('image_channel_count', 3) or 3),
        labels=list(params.get('labels', []) or []),
        has_anomaly=int(params.get('has_anomaly', 0) or 0),
        resize_mode=str(params.get('image_resize_mode', 'fit-shortest') or 'fit-shortest'),
        default_threshold=float(params.get('threshold', 0.0) or 0.0),
        raw=raw if isinstance(raw, dict) else {},
    )


class ModelRunner:
    """Owns the lifecycle of a single Edge Impulse ``.eim`` model process."""

    def __init__(self, model_path: str):
        """Store the path; the model process is not started until :meth:`start`."""
        self._model_path = model_path
        self._runner = None

    def start(self) -> ModelInfo:
        """
        Spawn the model process and return its parsed metadata.

        Raises ``ImportError`` if the Edge Impulse Linux SDK (or one of its
        dependencies) cannot be imported, and propagates any error raised while
        loading the model.
        """
        try:
            from edge_impulse_linux.runner import ImpulseRunner
        except ImportError as exc:  # pragma: no cover - depends on runtime env
            missing = getattr(exc, 'name', None)
            if missing and not missing.startswith('edge_impulse_linux'):
                extra = (' It also needs the system library `portaudio19-dev`.'
                         if missing == 'pyaudio' else '')
                raise ImportError(
                    'The Edge Impulse Linux SDK could not load because its '
                    f'dependency "{missing}" is missing; install it with '
                    f'`pip install {missing}`.{extra}'
                ) from exc
            raise ImportError(
                'The Edge Impulse Linux SDK is required at runtime. Install it '
                'with `pip install edge_impulse_linux`.'
            ) from exc

        self._runner = ImpulseRunner(self._model_path)
        raw = self._runner.init()
        return parse_model_info(raw)

    def classify(self, features) -> dict:
        """Run inference on a flat feature list and return the raw result dict."""
        if self._runner is None:
            raise RuntimeError('ModelRunner.start() must be called before classify()')
        return self._runner.classify(features)

    def stop(self) -> None:
        """Terminate the model process; safe to call more than once."""
        runner, self._runner = self._runner, None
        if runner is not None:
            try:
                runner.stop()
            except Exception:  # pragma: no cover - best-effort teardown
                pass
