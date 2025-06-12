from setuptools import setup
import os
from glob import glob

package_name = 'cr3_hand_control'

setup(
    name=package_name,
    version='1.0.0',
    package_dir={'': 'src'},
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrew Lloyd Holland',
    maintainer_email='andrewlloydholland@gmail.com',
    description='CR3 Hand Tracking Control System with Pose Recognition',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_recognition_node = cr3_hand_control.pose_recognition_node:main',
        ],
    },
)
