from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 1. RPLIDAR Driver (Standard ROS 2 package)
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyACM0',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),
        # 2. Perception Node (Managed by uv / pyproject.toml)
        ExecuteProcess(
            cmd=[
                '/opt/venv/bin/perception',
                '--ros-args',
                '-p',
                'min_range:=0.15',
                '-p',
                'max_range:=4.00',
                '-p',
                'cluster_thresh:=0.20',
                '-p',
                'max_cone_width:=0.30',
            ],
            output='screen',
        ),
    ])
