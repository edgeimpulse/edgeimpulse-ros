# edgeimpulse_ros

Minimal ROS2 wrapper to run the Edge Impulse Linux SDK example classifier and publish detections as ROS messages.

Features
- Imports the Edge Impulse Linux SDK and runs the classifier directly (no subprocess)
- Uses the camera via OpenCV and the SDK `ImageImpulseRunner`
- Publishes detections as JSON on topic `edgeimpulse/detections` (`std_msgs/String`)

Requirements
- ROS2 (Foxy/Galactic/Humble/etc.) with Python support
- Python 3
- OpenCV for Python (`opencv-python`)
- Edge Impulse linux SDK (see install instructions)

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
             -p camera:=0
```

What you'll get
- The node publishes JSON strings on `edgeimpulse/detections`. Each message has the form:

```json
{"boxes": [{"label":"joint","score":0.96,"x":56,"y":144,"w":16,"h":16}, ...]}
```

Notes & next steps
- You can adapt the parser if your SDK output format differs.
- If you prefer strongly-typed ROS messages, the node can be extended to publish a custom message type (requires adding msg files and message generation).
- The node imports the SDK directly; no subprocess is used. If you'd like a custom ROS message type for detections, I can add a `msg` and publishing logic.
# edgeimpulse-ros
ROS packages for edge impulse deployment
