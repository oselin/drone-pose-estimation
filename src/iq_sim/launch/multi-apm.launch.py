from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_nodes(context, *args, **kwargs):
    n_drones =  LaunchConfiguration('n_drones').perform(context)

    # Rest of your code remains unchanged
    apm_pluginlists_file = '/opt/ros/humble/share/mavros/launch/apm_pluginlists.yaml'
    apm_config_file = '/opt/ros/humble/share/mavros/launch/apm_config.yaml'

    nodes = []
    for i in range(1, int(n_drones)+1):

        # the first has to be 14551, but i starts from 1..
        udp_port_in = 14541 + i*10
        udp_port_out = udp_port_in + 4
        nodes.append(Node(
            package='mavros',
            executable='mavros_node',
            parameters=[
                {'fcu_url': f'udp://127.0.0.1:{udp_port_in}@%{udp_port_out}'},
                {'target_system_id': i},
                {'target_component_id': 1},
                {'fcu_protocol': 'v2.0'},
                apm_pluginlists_file,
                apm_config_file,
            ],
            namespace=f'/drone{i}',
            output='screen',
            respawn=True,
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([

        # Declare the 'n_drones' command-line argument with default value 1
        DeclareLaunchArgument(
            'n_drones',
            default_value='1',
            description='Number of drone nodes to create'
        ),

        OpaqueFunction(function=generate_nodes)
    ])

