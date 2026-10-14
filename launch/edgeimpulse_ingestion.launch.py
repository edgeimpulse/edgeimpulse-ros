"""
Launch the Edge Impulse ingestion node on an existing image topic.

Bring your own camera driver (``v4l2_camera``, ``usb_cam``, the Qualcomm QRB
camera, a bag file, ...) and point ``image_topic`` at the image it publishes.
Publish a ``std_msgs/String`` JSON payload (e.g. ``{"category": "training",
"label": "cat", "filename": "sample"}``) on the ``trigger`` topic to upload the
most recent frame to Edge Impulse.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description for the standalone ingestion node."""
    args = [
        DeclareLaunchArgument('api_key', default_value='',
                              description='Edge Impulse ingestion API key (required)'),
        DeclareLaunchArgument('namespace', default_value='',
                              description='Optional namespace for the node'),
        DeclareLaunchArgument('image_topic', default_value='image',
                              description='Input image topic to subscribe to'),
        DeclareLaunchArgument('image_transport', default_value='raw',
                              description='raw or compressed'),
        DeclareLaunchArgument('image_qos', default_value='sensor_data',
                              description='sensor_data | reliable | default'),
    ]

    node = Node(
        package='edgeimpulse_ros',
        executable='edgeimpulse_ingestion',
        name='edgeimpulse_ingestion',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'api_key': LaunchConfiguration('api_key'),
            'image_topic': LaunchConfiguration('image_topic'),
            'image_transport': LaunchConfiguration('image_transport'),
            'image_qos': LaunchConfiguration('image_qos'),
        }],
    )

    return LaunchDescription(args + [node])
