# edgeimpulse_ros

Minimal ROS2 wrapper to run an Edge Impulse `.eim` object detection model and publish bounding boxes as ROS messages.

Features
- Imports the Edge Impulse Linux SDK and runs the classifier directly (no subprocess)
- Uses the camera via the SDK `ImageImpulseRunner`
- Publishes detections as `vision_msgs/Detection2DArray`

Requirements
- ROS2 (Foxy/Galactic/Humble/etc.) with Python support
- Python 3
- A ROS install that includes `vision_msgs` and `geometry_msgs`
- Edge Impulse Linux Python SDK (`edge_impulse_linux`)
- OpenCV Python bindings (`cv2`) available to the same Python interpreter

On Ubuntu you can install ROS and OpenCV deps via:

```bash
sudo apt update
sudo apt install \
  ros-$ROS_DISTRO-vision-msgs \
  ros-$ROS_DISTRO-geometry-msgs \
  python3-opencv
```

Install and build

1. Clone this repo into your workspace `src`:

```bash
cd ~/your_ws/src
git clone <this-repo-url> edgeimpulse_ros
```

2. Install the Edge Impulse linux SDK (option A) or clone it (option B):

Install the Edge Impulse Linux Python SDK into your system Python (no venv):

```bash
python3 -m pip install --user edge_impulse_linux
```

If you prefer installing from the upstream repo:

```bash
python3 -m pip install --user git+https://github.com/edgeimpulse/linux-sdk-python.git
```

Note: Don’t place `linux-sdk-python` inside your ROS workspace `src/`.
Colcon will try to treat it as a workspace package. If you must keep it there, add a `COLCON_IGNORE` file.

3. Build the ROS workspace:

```bash
cd ~/your_ws
colcon build
source install/setup.bash
```

Run the node

The node imports the Edge Impulse SDK directly. Provide the model `.eim` file path and camera id.

```bash
# Example
ros2 run edgeimpulse_ros edgeimpulse_detector \
  --ros-args -p model_path:=/path/to/model.eim \
             -p camera:=0 \
             -p score_threshold:=0.5 \
             -p frame_id:=camera \
             -p detections_topic:=edgeimpulse/detections
```

Run with a launch file (recommended)

```bash
ros2 launch edgeimpulse_ros edgeimpulse_detector.launch.py \
  model_path:=/path/to/model.eim camera:=0 score_threshold:=0.5
```

Useful parameters
- `score_threshold` (default `0.5`): filter low-confidence boxes
- `log_detections` (default `true`): print detections to terminal
- `log_raw_bounding_boxes` (default `true`): print raw EI bounding boxes (before threshold)
- `log_frame_summary` (default `true`): print one summary line per frame (counts + timing)
- `publish_empty` (default `false`): publish empty `Detection2DArray` messages when no objects are found
- `status_period_sec` (default `5.0`): periodic status log interval
- `publish_timing` (default `true`): publish timing metadata per frame
- `timing_topic` (default `edgeimpulse/timing`): topic name for timing metadata
- `publish_count` (default `true`): publish per-frame detection count
- `count_topic` (default `edgeimpulse/count`): topic name for per-frame detection count (`std_msgs/Int32`)
- `fill_detection_header` (default `false`): copy array header into each `Detection2D` (more verbose)

What you'll get
- Topic: `edgeimpulse/detections` (configurable via `detections_topic`)
- Type: `vision_msgs/Detection2DArray`
- One message is published per inference frame, containing all bounding boxes for that frame
- Each detection contains a 2D bounding box and one hypothesis (class label + score)

Timing / metadata
- Topic: `edgeimpulse/timing` (configurable via `timing_topic`)
- Type: `std_msgs/String` (JSON)
- Contains per-frame timing (DSP/classification/anomaly/total in ms) and box counts

Count
- Topic: `edgeimpulse/count` (configurable via `count_topic`)
- Type: `std_msgs/Int32`
- Value: number of published detections in the most recent frame

Why are there pose/covariance fields?

This package publishes `vision_msgs/Detection2DArray`, and `vision_msgs` represents hypotheses as `ObjectHypothesisWithPose` (which includes a pose + covariance).
For 2D detectors, these fields are not meaningful and remain default/zero, but they still show up in `ros2 topic echo` output because they are part of the message definition.
If you want a minimal message without pose/covariance, we’d need to publish a custom message type (or a compact JSON topic).

Why is the timestamp repeated?

`Detection2DArray` has a header, and each `Detection2D` also has a header. By default we only fill the array header (`fill_detection_header:=false`) to reduce verbosity.

Notes & next steps
- The node expects the Edge Impulse Linux Python SDK to be installed (see install section).
- This is intentionally MVP: camera in -> detections out.

Changing default parameter values

Three easy options:

1) Override in the launch command line:

```bash
ros2 launch edgeimpulse_ros edgeimpulse_detector.launch.py \
  model_path:=/path/to/model.eim camera:=0 score_threshold:=0.3
```

2) Change defaults in the launch file (edit the `default_value=` fields):

- [edgeimpulse_detector.launch.py](launch/edgeimpulse_detector.launch.py)

3) Use a YAML params file:

```yaml
edgeimpulse_detector:
  ros__parameters:
    model_path: /path/to/model.eim
    camera: 0
    score_threshold: 0.5
    log_raw_bounding_boxes: false
    log_frame_summary: true
```

Then run:

```bash
ros2 run edgeimpulse_ros edgeimpulse_detector --ros-args --params-file params.yaml
```

