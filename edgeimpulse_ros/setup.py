from setuptools import setup

package_name = 'edgeimpulse_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='you@example.com',
    description='ROS2 wrapper to run Edge Impulse object detection',
    entry_points={
        'console_scripts': [
            'edgeimpulse_detector = edgeimpulse_ros.edgeimpulse_detector:main'
        ],
    },
)
