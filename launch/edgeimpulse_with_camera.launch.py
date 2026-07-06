"""
Demo: start a USB camera (``v4l2_camera``) and the Edge Impulse detector.

This is a convenience launch for quickly trying the package on a laptop or SBC
with a UVC webcam. In production you would instead run your real camera driver
and the standalone ``edgeimpulse_detector.launch.py``.

Requires the ``v4l2_camera`` package:
    sudo apt install ros-$ROS_DISTRO-v4l2-camera
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description for the camera + detector demo."""
    args = [
        DeclareLaunchArgument('model_path', default_value='',
                              description='Path to the Edge Impulse .eim model (required)'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0',
                              description='V4L2 device for the USB camera'),
        DeclareLaunchArgument('resize_mode', default_value='auto',
                              description='auto | squash | fit-shortest | fit-longest'),
        DeclareLaunchArgument('confidence_threshold', default_value='-1.0',
                              description='Min confidence; <0 uses the model default'),
        DeclareLaunchArgument('publish_debug_image', default_value='true',
                              description='Publish an annotated debug image'),
    ]

    camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_size': [640, 480],
        }],
    )

    detector = Node(
        package='edgeimpulse_ros',
        executable='edgeimpulse_detector',
        name='edgeimpulse_detector',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'image_topic': 'camera/image_raw',
            'image_transport': 'raw',
            'image_qos': 'sensor_data',
            'resize_mode': LaunchConfiguration('resize_mode'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'publish_debug_image': LaunchConfiguration('publish_debug_image'),
        }],
    )

    return LaunchDescription(args + [camera, detector])
