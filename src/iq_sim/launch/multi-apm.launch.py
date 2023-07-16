import os 

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    package_name = 'mavros'
    package_share_directory = get_package_share_directory(package_name)
    
    # ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://127.0.0.1:14561@14565 -p target_system_id:=2 -p target_component_id:=1 -p fcu_protocol:=v2.0 -r __ns:=/drone2
    mavros_node1 = Node(
        package='mavros',
        executable= 'mavros_node',
        # name='mavros',
        parameters=[
            {'fcu_url': 'udp://127.0.0.1:14551@14555'},
            # {'gcs_url': ''},
            {'target_system_id': 1},
            {'target_component_id': 1},
            {'fcu_protocol': 'v2.0'}
        ],
        namespace='/drone1',
        output='screen',
        respawn=True,
    )

    # mavros_node2 = Node(
    #     package='mavros',
    #     executable='mavros_node',
    #     # name='mavros',
    #     parameters=[
    #         {'fcu_url': 'udp://127.0.0.1:14561@14565'},
    #         # {'gcs_url': ''},
    #         {'target_system_id': 2},
    #         {'target_component_id': 1},
    #         {'fcu_protocol': 'v2.0'}
    #     ],
    #     namespace='/drone2',
    #     output='screen',
    #     respawn=True
    # )

    # mavros_node3 = Node(
    #     package='mavros',
    #     executable='mavros_node',
    #     # name='mavros',
    #     parameters=[
    #         {'fcu_url': 'udp://127.0.0.1:14571@14575'},
    #         # {'gcs_url': ''},
    #         {'target_system_id': 3},
    #         {'target_component_id': 1},
    #         {'fcu_protocol': 'v2.0'}
    #     ],
    #     namespace='/drone3',
    #     output='screen',
    #     respawn=True
    # )

    # Load parameter files for each node separately
    pluginlists_cmd1 = ExecuteProcess(
        cmd=['ros2', 'param', 'load', '/drone1', os.path.join(package_share_directory, 'launch/apm_pluginlists.yaml')],
        output='screen'
    )

    config_cmd1 = ExecuteProcess(
        cmd=['ros2', 'param', 'load', '/drone1', os.path.join(package_share_directory, 'launch/apm_config.yaml')],
        output='screen'
    )

    pluginlists_cmd2 = ExecuteProcess(
        cmd=['ros2', 'param', 'load', '/drone2/mavros', os.path.join(package_share_directory, 'launch/apm_pluginlists.yaml')],
        output='screen'
    )

    config_cmd2 = ExecuteProcess(
        cmd=['ros2', 'param', 'load', '/drone2/mavros', os.path.join(package_share_directory, 'launch/apm_config.yaml')],
        output='screen'
    )

    pluginlists_cmd3 = ExecuteProcess(
        cmd=['ros2', 'param', 'load', '/drone3/mavros', os.path.join(package_share_directory, 'launch/apm_pluginlists.yaml')],
        output='screen'
    )

    config_cmd3 = ExecuteProcess(
        cmd=['ros2', 'param', 'load', '/drone3/mavros', os.path.join(package_share_directory, 'launch/apm_config.yaml')],
        output='screen'
    )

    ld.add_action(mavros_node1)
    ld.add_action(pluginlists_cmd1)
    ld.add_action(config_cmd1)

    # ld.add_action(mavros_node2)
    # ld.add_action(pluginlists_cmd2)
    # ld.add_action(config_cmd2)

    # ld.add_action(mavros_node3)
    # ld.add_action(pluginlists_cmd3)
    # ld.add_action(config_cmd3)

    return ld

