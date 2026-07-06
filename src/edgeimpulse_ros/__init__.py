"""
ROS 2 integration for Edge Impulse Linux inference models.

This package turns an Edge Impulse ``.eim`` model into a camera-agnostic ROS 2
perception node. Images are consumed from a standard ``sensor_msgs/Image``
topic (produced by any camera driver) and results are published as idiomatic
``vision_msgs`` messages.
"""

__version__ = '1.0.0'
