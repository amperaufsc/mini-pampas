from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    sllidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        respawn=True,
        respawn_delay=1.5,
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyACM0',
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'inverted': True,  # LiDAR is mounted upside down; flips scan so +y stays left
            'angle_compensate': True,
            'scan_mode': '',
        }],
    )

    perception = ExecuteProcess(
        cmd=[
            '/usr/bin/python3',
            '/mini_pampas/src/mini_pampas/perception/ros/perceptionNode.py',
            '--ros-args',
            '-p', 'min_range:=0.15',
            '-p', 'max_range:=1.00',
            '-p', 'cluster_thresh:=0.20',
            '-p', 'max_cone_width:=0.20',
        ],
        output='screen',
    )

    return LaunchDescription([
        sllidar,
        TimerAction(period=3.0, actions=[perception]),
    ])