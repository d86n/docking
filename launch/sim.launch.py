from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_docking = get_package_share_directory('docking')

    gz_sim_launch = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    world = PathJoinSubstitution([pkg_docking, 'worlds', 'docking_world.sdf'])

    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([gz_sim_launch]), launch_arguments={'gz_args': world}.items())

    ld = LaunchDescription()
    ld.add_action(gazebo)

    return ld