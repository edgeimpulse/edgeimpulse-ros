"""
Launch the Edge Impulse detector on an existing image topic.

This launch file starts only the inference node. Bring your own camera driver
(``v4l2_camera``, ``usb_cam``, the Qualcomm QRB camera, a bag file, ...) and
point ``image_topic`` at the image it publishes. For a one-command demo that
also starts a USB camera, see ``edgeimpulse_with_camera.launch.py``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description for the standalone detector node."""
    args = [
        DeclareLaunchArgument('model_path', default_value='',
                              description='Path to the Edge Impulse .eim model (required)'),
        DeclareLaunchArgument('namespace', default_value='',
                              description='Optional namespace for the node'),
        DeclareLaunchArgument('image_topic', default_value='image',
                              description='Input image topic to subscribe to'),
        DeclareLaunchArgument('image_transport', default_value='raw',
                              description='raw or compressed'),
        DeclareLaunchArgument('image_qos', default_value='sensor_data',
                              description='sensor_data | reliable | default'),
        DeclareLaunchArgument('resize_mode', default_value='auto',
                              description='auto | squash | fit-shortest | fit-longest'),
        DeclareLaunchArgument('confidence_threshold', default_value='-1.0',
                              description='Min confidence; <0 uses the model default'),
        DeclareLaunchArgument('publish_debug_image', default_value='false',
                              description='Publish an annotated debug image'),
        DeclareLaunchArgument('frame_id_override', default_value='',
                              description='Override the source image frame_id if set'),
    ]

    node = Node(
        package='edgeimpulse_ros',
        executable='edgeimpulse_detector',
        name='edgeimpulse_detector',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'image_topic': LaunchConfiguration('image_topic'),
            'image_transport': LaunchConfiguration('image_transport'),
            'image_qos': LaunchConfiguration('image_qos'),
            'resize_mode': LaunchConfiguration('resize_mode'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'publish_debug_image': LaunchConfiguration('publish_debug_image'),
            'frame_id_override': LaunchConfiguration('frame_id_override'),
        }],
    )

    return LaunchDescription(args + [node])
