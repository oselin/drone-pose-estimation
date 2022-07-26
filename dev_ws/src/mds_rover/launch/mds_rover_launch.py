import os


from asyncio.log import logger
from launch_ros.actions import Node
from launch import LaunchDescription

package_name = 'mds_rover'


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mds_rover'),
        'config',
        'config.yaml'
    )
    anchor_node = Node(
        package='mds_rover',
        executable='anchor',
        namespace='anchor',
        output='screen',
        parameters=[config],
    )
    rover_nodes = [
        Node(
            package='mds_rover',
            executable='rover',
            namespace='rover{i}',
            output='screen',
            parameters=[config]
        )
        for i in range(4)
    ]
    return LaunchDescription([
        Node(
            package='mds_rover',
            executable='anchor',
            namespace='anchor',
            name='anchor',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='mds_rover',
            namespace='rover1',
            executable='rover',
            name='rover',
            output='screen',
            parameters=[config]
        )])
