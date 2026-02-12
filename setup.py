from setuptools import find_packages, setup

package_name = 'edgeimpulse_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/edgeimpulse_ros']),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', [
            'launch/edgeimpulse_detector.launch.py',
            'launch/edgeimpulse_detector_with_tracker.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='edgeimpulse_ros contributors',
    maintainer_email='noreply@example.com',
    description='ROS2 wrapper to run Edge Impulse object detection',
    entry_points={
        'console_scripts': [
            'edgeimpulse_detector = edgeimpulse_ros.edgeimpulse_detector:main',
            'edgeimpulse_pt_track = edgeimpulse_ros.edgeimpulse_pt_track:main'
        ],
    },
)
