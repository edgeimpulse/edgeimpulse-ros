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

What you'll get
- Topic: `edgeimpulse/detections` (configurable via `detections_topic`)
- Type: `vision_msgs/Detection2DArray`
- Each detection contains a 2D bounding box and one hypothesis (class label + score)

Notes & next steps
- The node expects the Edge Impulse Linux Python SDK to be installed (see install section).
- This is intentionally MVP: camera in -> detections out.
# edgeimpulse-ros
