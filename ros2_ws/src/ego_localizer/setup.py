import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ego_localizer'

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
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Ricardo de Azambuja',
    maintainer_email='ricardo.azambuja@gmail.com',
    description='Ego-pose EKF node wrapping fusion_core (IMU + odometry fusion). '
                'PLAN milestone 3.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ego_localizer = ego_localizer.node:main',
        ],
    },
)
