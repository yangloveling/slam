from setuptools import setup
import os
from glob import glob

package_name = 'hi13s2_imu'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='he',
    maintainer_email='he@todo.todo',
    description='HI13S2 IMU driver',
    license='Apache License 2.0',

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],

        entry_points={
        'console_scripts': [
            'hi13s2_node = hi13s2_imu.hi13s2_node:main',
            'yaw_publisher = hi13s2_imu.yaw_publisher:main',
             
        ],
    },
)
