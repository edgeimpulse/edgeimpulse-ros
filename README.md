# edgeimpulse_ros

Minimal ROS2 wrapper to run an Edge Impulse `.eim` object detection model and publish bounding boxes as ROS messages.

Features
- Imports the Edge Impulse Linux SDK and runs the classifier directly (no subprocess)
- Uses the camera via the SDK `ImageImpulseRunner`
- Publishes detections as `vision_msgs/Detection2DArray`

Requirements
- ROS2 (Foxy/Galactic/Humble/etc.) with Python support
- Python 3
- Edge Impulse linux SDK (see install instructions)
- A ROS install that includes `vision_msgs` and `geometry_msgs`

On Ubuntu you can install `vision_msgs` via:

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-vision-msgs
```

Install and build

1. Clone this repo into your workspace `src`:

```bash
cd ~/your_ws/src
git clone <this-repo-url> edgeimpulse_ros
```

2. Install the Edge Impulse linux SDK (option A) or clone it (option B):

Option A - install from GitHub (recommended):

```bash
pip3 install --user git+https://github.com/edgeimpulse/linux-sdk-python.git
```

Option B - clone and editable install:

```bash
git clone https://github.com/edgeimpulse/linux-sdk-python.git ~/repos/linux-sdk-python
cd ~/repos/linux-sdk-python
pip3 install --user -e .
```

Note: Don’t place `linux-sdk-python` inside your ROS workspace `src/` unless you add a `COLCON_IGNORE` file.
Colcon will try to build it and fail to parse its `install_requires`.

Virtualenv note

ROS 2 console scripts installed by `colcon build` are tied to the Python interpreter used during the build.
If you create/activate a venv *after* building, `ros2 run` may still use system Python and won’t see packages installed in the venv.

Fix options:
- Activate your venv first, then rebuild your workspace (`rm -rf build install log && colcon build`).
- Or install the Edge Impulse SDK into the system Python that ROS is using.

If you installed the EI SDK with `pip --user` but ROS still can’t import it, check whether your environment disables user site-packages:

```bash
echo $PYTHONNOUSERSITE
```

If it prints `1`, unset it (or start a clean shell) and try again.

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
ROS packages for edge impulse deployment
