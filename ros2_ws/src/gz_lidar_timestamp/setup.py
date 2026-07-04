import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'gz_lidar_timestamp'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ricardo de Azambuja',
    maintainer_email='ricardo.azambuja@gmail.com',
    description='Append per-point relative timestamps to a Gazebo gpu_lidar '
                'PointCloud2 so lidar odometry can deskew.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'timestamp_injector = gz_lidar_timestamp.timestamp_injector:main',
        ],
    },
)
