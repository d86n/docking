from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('x'),
    DeclareLaunchArgument('y'),
    DeclareLaunchArgument('yaw'),
]

def generate_launch_description():
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    pkg_irobot_create_gz_bringup = get_package_share_directory(
        'irobot_create_gz_bringup')

    create3_spawn_launch = PathJoinSubstitution(
        [pkg_irobot_create_gz_bringup, 'launch', 'create3_spawn.launch.py'])

    create3_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create3_spawn_launch]),
        launch_arguments=[
            ('x', x),
            ('y', y),
            ('yaw', yaw)
        ]
    )

    return LaunchDescription(ARGUMENTS + [
        create3_spawn
    ])