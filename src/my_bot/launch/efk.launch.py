from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Madgwick filter
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'publish_tf': False
            }],
            remappings=[
                ('imu/data_raw', '/imu/mpu6050'),
                ('imu/data', '/imu/madgwick_out')
            ]
        ),

        # Node Python tự viết (phải chạy bằng 'python3')
        Node(
            package='my_bot',            # Gói chứa file .py
            executable='imu_gate_node.py',  # Chỉ rõ file .py luôn
            name='imu_gate_node',
            output='screen',
            prefix='python3',            # ROS2 gọi bằng python3 trực tiếp
            remappings=[
                ('imu/data_raw', '/imu/madgwick_out'),
                ('imu/data', '/imu/filtered')
            ]
        ),


        # EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                '/home/jetson/dev_ws/src/my_bot/config/robot_localization.yaml'
            ],
            remappings=[
                ('/odometry/filtered', '/odometry/filtered')
            ]
        )
    ])
