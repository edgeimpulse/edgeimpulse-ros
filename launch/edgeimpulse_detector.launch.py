from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path = LaunchConfiguration('model_path')
    camera = LaunchConfiguration('camera')
    score_threshold = LaunchConfiguration('score_threshold')
    frame_id = LaunchConfiguration('frame_id')
    detections_topic = LaunchConfiguration('detections_topic')
    timing_topic = LaunchConfiguration('timing_topic')
    publish_timing = LaunchConfiguration('publish_timing')
    count_topic = LaunchConfiguration('count_topic')
    publish_count = LaunchConfiguration('publish_count')
    publish_empty = LaunchConfiguration('publish_empty')
    log_detections = LaunchConfiguration('log_detections')
    log_raw_bounding_boxes = LaunchConfiguration('log_raw_bounding_boxes')
    log_frame_summary = LaunchConfiguration('log_frame_summary')
    fill_detection_header = LaunchConfiguration('fill_detection_header')
    status_period_sec = LaunchConfiguration('status_period_sec')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'model_path',
                default_value='',
                description='Path to Edge Impulse .eim model file (required)',
            ),
            DeclareLaunchArgument(
                'camera',
                default_value='0',
                description='V4L2 camera index to use',
            ),
            DeclareLaunchArgument(
                'score_threshold',
                default_value='0.5',
                description='Minimum score to publish a detection',
            ),
            DeclareLaunchArgument(
                'frame_id',
                default_value='camera',
                description='Header frame_id for published detections',
            ),
            DeclareLaunchArgument(
                'detections_topic',
                default_value='edgeimpulse/detections',
                description='Topic name for Detection2DArray output',
            ),
            DeclareLaunchArgument(
                'timing_topic',
                default_value='edgeimpulse/timing',
                description='Topic name for per-frame timing metadata (std_msgs/String JSON)',
            ),
            DeclareLaunchArgument(
                'publish_timing',
                default_value='true',
                description='Publish per-frame timing metadata',
            ),
            DeclareLaunchArgument(
                'count_topic',
                default_value='edgeimpulse/count',
                description='Topic name for per-frame detection count (std_msgs/Int32)',
            ),
            DeclareLaunchArgument(
                'publish_count',
                default_value='true',
                description='Publish per-frame detection count',
            ),
            DeclareLaunchArgument(
                'publish_empty',
                default_value='false',
                description='Publish empty Detection2DArray when there are no detections',
            ),
            DeclareLaunchArgument(
                'log_detections',
                default_value='false',
                description='Log published detections to terminal',
            ),
            DeclareLaunchArgument(
                'log_raw_bounding_boxes',
                default_value='true',
                description='Log raw bounding boxes from EI output (before threshold)',
            ),
            DeclareLaunchArgument(
                'log_frame_summary',
                default_value='true',
                description='Log one summary line per frame with counts + timing',
            ),
            DeclareLaunchArgument(
                'fill_detection_header',
                default_value='false',
                description='Copy array header into each Detection2D (more verbose output)',
            ),
            DeclareLaunchArgument(
                'status_period_sec',
                default_value='5.0',
                description='Periodic status logging interval in seconds',
            ),
            Node(
                package='edgeimpulse_ros',
                executable='edgeimpulse_detector',
                name='edgeimpulse_detector',
                output='screen',
                parameters=[
                    {
                        'model_path': model_path,
                        'camera': camera,
                        'score_threshold': score_threshold,
                        'frame_id': frame_id,
                        'detections_topic': detections_topic,
                        'timing_topic': timing_topic,
                        'publish_timing': publish_timing,
                        'count_topic': count_topic,
                        'publish_count': publish_count,
                        'publish_empty': publish_empty,
                        'log_detections': log_detections,
                        'log_raw_bounding_boxes': log_raw_bounding_boxes,
                        'log_frame_summary': log_frame_summary,
                        'fill_detection_header': fill_detection_header,
                        'status_period_sec': status_period_sec,
                    }
                ],
            ),
        ]
    )
