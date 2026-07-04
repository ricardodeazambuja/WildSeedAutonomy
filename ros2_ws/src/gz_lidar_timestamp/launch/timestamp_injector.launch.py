from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument('input_topic', default_value='/cloud_in'),
        DeclareLaunchArgument('output_topic', default_value='/cloud_with_time'),
        DeclareLaunchArgument('scan_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('profile', default_value='velodyne',
                              description='velodyne (time/float32 s) | ouster (t/uint32 ns)'),
        DeclareLaunchArgument('method', default_value='auto',
                              description='auto | column | azimuth'),
    ]
    node = Node(
        package='gz_lidar_timestamp',
        executable='timestamp_injector',
        name='gz_lidar_timestamp',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'scan_rate_hz': ParameterValue(LaunchConfiguration('scan_rate_hz'),
                                           value_type=float),
            'profile': LaunchConfiguration('profile'),
            'method': LaunchConfiguration('method'),
        }],
    )
    return LaunchDescription(args + [node])
