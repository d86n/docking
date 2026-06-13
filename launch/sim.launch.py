import os
from ament_index_python.packages import get_package_share_directory

from irobot_create_common_bringup.namespace import GetNamespacedName
from irobot_create_common_bringup.offset import OffsetParser, RotationalOffsetX, RotationalOffsetY

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # ==========================================
    # 1. KHỞI TẠO BẢN ĐỒ GAZEBO
    # ==========================================
    docking_pkg = get_package_share_directory('docking')
    world_file_path = os.path.join(docking_pkg, 'worlds', 'docking.sdf')

    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file_path}'}.items()
    )

    # ==========================================
    # 2. CHUẨN BỊ ĐƯỜNG DẪN
    # ==========================================
    pkg_turtlebot4_gz_bringup = get_package_share_directory('turtlebot4_gz_bringup')
    pkg_turtlebot4_description = get_package_share_directory('turtlebot4_description')
    pkg_irobot_create_common_bringup = get_package_share_directory('irobot_create_common_bringup')
    pkg_irobot_create_gz_bringup = get_package_share_directory('irobot_create_gz_bringup')

    turtlebot4_ros_gz_bridge_launch = PathJoinSubstitution([pkg_turtlebot4_gz_bringup, 'launch', 'ros_gz_bridge.launch.py'])
    turtlebot4_node_launch = PathJoinSubstitution([pkg_turtlebot4_gz_bringup, 'launch', 'turtlebot4_nodes.launch.py'])
    create3_nodes_launch = PathJoinSubstitution([pkg_irobot_create_common_bringup, 'launch', 'create3_nodes.launch.py'])
    create3_gz_nodes_launch = PathJoinSubstitution([pkg_irobot_create_gz_bringup, 'launch', 'create3_gz_nodes.launch.py'])
    robot_description_launch = PathJoinSubstitution([pkg_turtlebot4_description, 'launch', 'robot_description.launch.py'])
    turtlebot4_node_yaml_file = PathJoinSubstitution([pkg_turtlebot4_gz_bringup, 'config', 'turtlebot4_node.yaml'])

    # ==========================================
    # 3. KHAI BÁO CÁC BIẾN (CHUẨN ROS 2 LAUNCH)
    # ==========================================
    # Sử dụng DeclareLaunchArgument để định nghĩa biến nội bộ
    arg_namespace = DeclareLaunchArgument('namespace', default_value='')
    arg_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    arg_model = DeclareLaunchArgument('model', default_value='standard')
    
    arg_x = DeclareLaunchArgument('x', default_value='1.0')
    arg_y = DeclareLaunchArgument('y', default_value='0.0')
    arg_z = DeclareLaunchArgument('z', default_value='0.0')
    arg_yaw = DeclareLaunchArgument('yaw', default_value='0.0')

    # Đọc giá trị vào các object LaunchConfiguration
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    x, y, z, yaw = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z'), LaunchConfiguration('yaw')

    robot_name = GetNamespacedName(namespace, 'turtlebot4')

    z_robot = OffsetParser(z, -0.0025)

    # ==========================================
    # 4. GỘP CÁC NODE VÀ BRIDGES
    # ==========================================
    spawn_robot_group_action = GroupAction([
        PushRosNamespace(namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_description_launch]),
            launch_arguments=[('model', model), ('use_sim_time', use_sim_time)]
        ),

        Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=['-name', robot_name, '-x', x, '-y', y, '-z', z_robot, '-Y', yaw, '-topic', 'robot_description']
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/model/turtlebot4/sensor/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image'],
            remappings=[('/model/turtlebot4/sensor/rgbd_camera/image', '/oakd/rgb/preview/image_raw')],
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot4_ros_gz_bridge_launch]),
            launch_arguments=[('model', model), ('robot_name', robot_name), ('namespace', namespace)]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot4_node_launch]),
            launch_arguments=[('model', model), ('param_file', turtlebot4_node_yaml_file)]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([create3_nodes_launch]),
            launch_arguments=[('namespace', namespace)]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([create3_gz_nodes_launch]),
            launch_arguments=[('robot_name', robot_name)]
        ),

        Node(
            name='rplidar_stf', package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0', '0', '0', '0.0', 'rplidar_link', [robot_name, '/rplidar_link/rplidar']],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),
        Node(
            name='camera_stf', package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0', '1.5707', '-1.5707', '0', 'oakd_rgb_camera_optical_frame', [robot_name, '/oakd_rgb_camera_frame/rgbd_camera']],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),
    ])

    ld = LaunchDescription()
    
    # Nạp các khai báo biến vào LaunchDescription
    ld.add_action(arg_namespace)
    ld.add_action(arg_use_sim_time)
    ld.add_action(arg_model)
    ld.add_action(arg_x)
    ld.add_action(arg_y)
    ld.add_action(arg_z)
    ld.add_action(arg_yaw)
    
    # Nạp các action chính
    ld.add_action(gz_sim)
    ld.add_action(spawn_robot_group_action)
    return ld