from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dynamic',
    maintainer_email='Nargeskari84@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motion_controller_node = robot_estimation.motion_controller_node:main',
            'odometry_publisher_node = robot_estimation.odometry_publisher_node:main',
            'lowpass_imu_node = robot_estimation.lowpass_imu_node:main',
            'complementary_filter_node = robot_estimation.complementary_filter_node:main',
            'wheel_rpm_publisher = robot_estimation.wheel_rpm_publisher:main',
        ],
    },
)