# edgeimpulse_ros

Minimal ROS2 wrapper to run the Edge Impulse Linux SDK example classifier and publish detections as ROS messages.

Features
- Runs the SDK `examples/image/classify.py` as a subprocess
- Parses its stdout for bounding boxes
- Publishes detections as JSON on topic `edgeimpulse/detections` (`std_msgs/String`)

Requirements
- ROS2 (Foxy/Galactic/Humble/etc.) with Python support
- Python 3
- Edge Impulse linux SDK (the repository `https://github.com/edgeimpulse/linux-sdk-python`) cloned locally (see instructions)

Why subprocess?
This wrapper uses the existing example script output format (text) to avoid hard dependency on SDK internals and to remain compatible with the user's working example.

Install and build

1. Clone this repo into your workspace `src`:

```bash
cd ~/your_ws/src
git clone <this-repo-url> edgeimpulse_ros
```

2. Clone (or download) the Edge Impulse linux SDK somewhere on the machine. Example:

```bash
cd ~/repos
git clone https://github.com/edgeimpulse/linux-sdk-python.git
```

3. Build the ROS workspace:

```bash
cd ~/your_ws
colcon build
source install/setup.bash
```

Run the node

The node runs the SDK example `examples/image/classify.py`. Provide the full path to that script and the model `.eim` file.

```bash
# Example
ros2 run edgeimpulse_ros edgeimpulse_detector \
  --ros-args -p sdk_script:=/home/user/repos/linux-sdk-python/examples/image/classify.py \
             -p model_path:=/path/to/model.eim \
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
- To avoid the subprocess approach, the node can be reimplemented by importing the SDK directly; if you want that, tell me and I will update the node.
# edgeimpulse-ros
ROS packages for edge impulse deployment
