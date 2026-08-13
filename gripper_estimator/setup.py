from setuptools import find_packages, setup

package_name = 'gripper_estimator'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='kalanaratnayake95@gmail.com',
    description='Force calibration and retention-force estimation tools for the gripper stack',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gripping_force_estimate = gripper_estimator.gripping_force_estimate:main',
            'retention_force_estimate = gripper_estimator.retention_force_estimate:main',
        ],
    },
)