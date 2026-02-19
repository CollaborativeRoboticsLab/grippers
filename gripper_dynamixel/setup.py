from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'gripper_dynamixel'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'dynamixel-sdk'],
    zip_safe=True,
    maintainer='kalana',
    maintainer_email='kalanaratnayake95@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gripper_dynamixel_action_node = gripper_dynamixel.gripper_action_node:main',
        ],
    },
)
