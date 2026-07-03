from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'edgeimpulse_ros'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Edge Impulse',
    maintainer_email='hello@edgeimpulse.com',
    description='Run Edge Impulse image models in ROS 2 and publish vision_msgs results.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'edgeimpulse_detector = edgeimpulse_ros.edgeimpulse_detector:main',
        ],
    },
)
