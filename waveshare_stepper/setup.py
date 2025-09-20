from setuptools import setup
import os
from glob import glob

package_name = 'waveshare_stepper'

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
        'RPi.GPIO',
    ],
    zip_safe=True,
    maintainer='Christophe Foyer',
    maintainer_email='your.email@example.com',
    description='Waveshare stepper motor driver for ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'waveshare_stepper_node = waveshare_stepper.waveshare_stepper_node:main',
        ],
    },
)
