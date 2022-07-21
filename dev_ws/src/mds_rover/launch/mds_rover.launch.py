from ast import arguments
from launch_ros.actions import Node

from launch import LaunchDescription
from std_msgs.msg import Float32MultiArray


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mds_rover',
            namespace='anchor',
            executable='anchor',
            name='anchor',
            parameters=[{'position': {"x":1.0,"y":1.0,"z":1.0},'number_of_drones': 1}]
        ),
        Node(
            package='mds_rover',
            namespace='rover1',
            executable='rover',
            name='rover1',
            parameters=[{   'position': {'x':1.0,'y':0.0,'z':0.0},
                            'index': 1,
                            'dists': [2 , 1]
                        }]
        )#,
        #Node(
        #    package='mds_rover',
        #    namespace='rover2',
        #    executable='rover',
        #    name='rover2',
        #    parameters=[{   'position': {'x':2.0,'y':2.0,'z':0.0}, 
        #                    'index': 2,
        #                    'dists': {1.0,1.0,1.0,1.0}
        #                }]
        #),
        #Node(
        #    package='mds_rover',
        #    namespace='rover3',
        #    executable='rover',
        #    name='rover3',
        #    parameters=[{   'position': {'x':3.0,'y':3.0,'z':0.0},
        #                    'index': 3,
        #                    'dists': {1.0,1.0,1.0,1.0}
        #                }]
        #)
    ])
