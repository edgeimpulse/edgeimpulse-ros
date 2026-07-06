^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package edgeimpulse_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-07-03)
------------------
* Rewrote the package around a camera-agnostic design: the node subscribes to a
  ``sensor_msgs/Image`` topic from any driver instead of opening a camera.
* Added an encoding normaliser that decodes ``bgr8`` / ``rgb8`` / ``mono8`` /
  ``mono16`` / ``bgra8`` / ``rgba8`` / ``yuyv`` / ``uyvy`` and ``nv12`` /
  ``nv21`` without ``cv_bridge`` (fixes the Qualcomm QRB NV12 workflow).
* Published idiomatic ``vision_msgs`` results (``Detection2DArray``,
  ``Classification``), anomaly scores, latched ``VisionInfo`` / ``LabelInfo``
  and ``diagnostic_msgs/DiagnosticArray``.
* Propagated the source image ``header`` (timestamp and ``frame_id``) to every
  output so TF lookups and sensor fusion keep working.
* Mapped detections from model-input space back to original image coordinates
  (squash / fit-shortest / fit-longest aware).
* Added an optional annotated debug image, configurable QoS and a
  latest-frame-wins worker so slow hardware never builds a backlog.
* Added unit tests for the pure image/conversion logic and a node smoke test,
  plus ``flake8`` / ``pep257`` linting.
