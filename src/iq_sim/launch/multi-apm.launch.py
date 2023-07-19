import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    # Default configuration
    apm_pluginlists_file = '/opt/ros/humble/share/mavros/launch/apm_pluginlists.yaml'
    apm_config_file = '/opt/ros/humble/share/mavros/launch/apm_config.yaml'

    # Command: ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://127.0.0.1:14561@14565 -p target_system_id:=2 -p target_component_id:=1 -p fcu_protocol:=v2.0 -r __ns:=/drone2
    n_drones = 2
    for i in range(1, n_drones+1):
        # the first has to be 14551, but i starts from 1..
        udp_port = 14541 + i*10
        ld.add_action(Node(
            package='mavros',
            executable='mavros_node',
            # name="mavros",
            parameters=[
                # {'fcu_url': 'tcp://localhost:5763@5762'},
                {'fcu_url': 'udp://127.0.0.1:%i@%i' % (udp_port, udp_port+4)},
                # {'gcs_url' : 'udp://@'},
                # {'gcs_url': ''},
                {'target_system_id': i},
                {'target_component_id': 1},
                {'fcu_protocol': 'v2.0'},
                apm_pluginlists_file,
                apm_config_file,
            ],
            namespace='/drone%i' % i,
            output='screen',
            respawn=True,
            # remappings=[
            #     ("*/", "/mavros")
            # ]
        ))

    return ld

    # package_name = 'mavros'
    # package_share_directory = get_package_share_directory(package_name)
