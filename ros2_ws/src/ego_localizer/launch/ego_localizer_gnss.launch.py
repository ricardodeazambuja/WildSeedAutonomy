"""Launch ego_localizer in the GPS-denied keystone configuration (relative odom + GNSS)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('ego_localizer'),
                       'config', 'ego_localizer_gnss.yaml')
    return LaunchDescription([
        Node(package='ego_localizer', executable='ego_localizer',
             name='ego_localizer', output='screen', parameters=[cfg]),
    ])
