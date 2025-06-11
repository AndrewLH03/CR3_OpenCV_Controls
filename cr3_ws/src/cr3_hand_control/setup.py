from setuptools import setup
import os
from glob import glob

package_name = 'cr3_hand_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrew Lloyd Holland',
    maintainer_email='andrewlloydholland@gmail.com',
    description='CR3 Hand Tracking Control System - Advanced robotic arm control with real-time hand tracking',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parameter_manager = scripts.parameter_manager:main',
            'diagnostics_monitor = scripts.diagnostics_monitor:main',
        ],
    },
)
