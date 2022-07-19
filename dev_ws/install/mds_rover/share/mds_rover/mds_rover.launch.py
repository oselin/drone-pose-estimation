from ast import arguments
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mds_rover',
            namespace='anchor',
            executable='anchor',
            name='anchor',
            parameters=[{'position': {"x":1.0,"y":1.0,"z":1.0},'number_of_drones': 3}]
        ),
        Node(
            package='mds_rover',
            namespace='rover1',
            executable='rover',
            name='rover1',
            parameters=[{'position': {'x':1.0,'y':1.0,'z':0.0},'index': 1}]
        ),
        Node(
            package='mds_rover',
            namespace='rover2',
            executable='rover',
            name='rover2',
            parameters=[{'position': {'x':2.0,'y':2.0,'z':0.0},'index': 2}]
        ),
        Node(
            package='mds_rover',
            namespace='rover3',
            executable='rover',
            name='rover3',
            parameters=[{'position': {'x':3.0,'y':3.0,'z':0.0},'index': 3}]
        )
    ])
