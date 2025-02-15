import os
from setuptools import setup

package_name = 'arduino_uart'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 Node for Arduino Serial Communication with MPU6050',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_node = arduino_uart.serial_node:main',
        ],
    },
)

