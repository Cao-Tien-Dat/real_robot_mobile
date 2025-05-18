import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode

def generate_launch_description():
    # Gói chứa URDF robot
    my_bot_pkg = 'my_bot'
    lidar_pkg = 'ydlidar_ros2_driver'

    # URDF + TF từ my_bot
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(my_bot_pkg),
                'launch',
                'rsp.launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'use_ros2_control': 'false'
        }.items()
    )

    # Node UART điều khiển động cơ
    driver_uart_node = Node(
        package='driver_uart',
        executable='driver_uart',
        name='driver_uart_node',
        output='screen',
        emulate_tty=True
    )

    # Tham số file cấu hình LiDAR
    lidar_params_file = LaunchConfiguration('params_file')
    lidar_params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory(lidar_pkg),
            'params',
            'X4-Pro.yaml'
        ),
        description='File cấu hình cho YDLidar'
    )

    # Node driver LiDAR
    lidar_node = LifecycleNode(
        package=lidar_pkg,
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[lidar_params_file],
        namespace='/',
    )

    # Static TF từ base_link -> laser_frame
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=[
            '0.1', '0', '0.13',  # xyz (x=0.1m, z=0.063m)
            '0', '0', '0', '1',   # Quaternion (identity)
            'base_link', 'laser_frame'
        ]
    )

    return LaunchDescription([
        lidar_params_declare,
        rsp,
        driver_uart_node,
        lidar_node,
        static_tf
    ])
