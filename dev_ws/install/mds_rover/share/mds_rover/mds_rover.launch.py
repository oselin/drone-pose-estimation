import os
from ament_index_python.packages import get_package_share_directory
from ast import arguments
from launch_ros.actions import Node
from launch import LaunchDescription
from std_msgs.msg import Float32MultiArray


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mds_rover'),
        'config.yaml'
    )
    return LaunchDescription([
        Node(
            package='mds_rover',
            namespace='rover1',
            executable='rover',
            name='rover',
            output='screen',
            parameters=[config]
        ),
        Node(
            package='mds_rover',
            namespace='rover2',
            executable='rover',
            name='rover',
            output='screen',
            parameters=[config]
        ),
        Node(
            package='mds_rover',
            namespace='rover3',
            executable='rover',
            name='rover',
            output='screen',
            parameters=[config]
        ),
        Node(
            package='mds_rover',
            namespace='rover4',
            executable='rover',
            name='rover',
            output='screen',
            parameters=[config]
        ),
        Node(
            package='mds_rover',
            executable='anchor',
            namespace='anchor',
            name='anchor',
            output='screen',
            parameters=[config],
        )
    ])
