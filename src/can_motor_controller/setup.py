"""Package setup for can_motor_controller."""

from setuptools import setup

package_name = 'can_motor_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS2 CAN motor controller with feedback',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'three_axis_can_sender = can_motor_controller.three_axis_can_sender:main',
            'dynamic_target = can_motor_controller.dynamic_target:main',
        ],
    },
)
