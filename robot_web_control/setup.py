from setuptools import setup
import os
from glob import glob

package_name = 'robot_web_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs', 
        'geometry_msgs',
        'cv_bridge',
        'opencv-python',
        'flask',
        'flask-socketio',
        'python-socketio',
    ],
    zip_safe=True,
    maintainer='Christophe Foyer',
    maintainer_email='your.email@example.com',
    description='ROS2 robot control node with web interface',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_web_control = robot_web_control.robot_web_control_node:main',
        ],
    },
)
