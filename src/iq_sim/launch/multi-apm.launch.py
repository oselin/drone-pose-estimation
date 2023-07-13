from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    mavros_pkg_share_dir = get_package_share_directory('mavros')

    mavros_node1 = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        parameters=[
            {'fcu_url': 'udp://127.0.0.1:14551@14555'},
            {'gcs_url': ''},
            {'target_system_id': '1'},
            {'target_component_id': '1'},
            {'fcu_protocol': 'v2.0'}
        ],
        remappings=[
            ('/drone1/apm_pluginlists.yaml', mavros_pkg_share_dir + '/launch/apm_pluginlists.yaml'),
            ('/drone1/apm_config.yaml', mavros_pkg_share_dir + '/launch/apm_config.yaml')
        ],
        namespace='/drone1',
        output='screen',
        respawn=True
    )
    ld.add_action(mavros_node1)

    mavros_node2 = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        parameters=[
            {'fcu_url': 'udp://127.0.0.1:14561@14565'},
            {'gcs_url': ''},
            {'target_system_id': '2'},
            {'target_component_id': '1'},
            {'fcu_protocol': 'v2.0'}
        ],
        remappings=[
            ('/drone2/apm_pluginlists.yaml', mavros_pkg_share_dir + '/launch/apm_pluginlists.yaml'),
            ('/drone2/apm_config.yaml', mavros_pkg_share_dir + '/launch/apm_config.yaml')
        ],
        namespace='/drone2',
        output='screen',
        respawn=True
    )
    ld.add_action(mavros_node2)

    mavros_node3 = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        parameters=[
            {'fcu_url': 'udp://127.0.0.1:14571@14575'},
            {'gcs_url': ''},
            {'target_system_id': '3'},
            {'target_component_id': '1'},
            {'fcu_protocol': 'v2.0'}
        ],
        remappings=[
            ('/drone3/apm_pluginlists.yaml', mavros_pkg_share_dir + '/launch/apm_pluginlists.yaml'),
            ('/drone3/apm_config.yaml', mavros_pkg_share_dir + '/launch/apm_config.yaml')
        ],
        namespace='/drone3',
        output='screen',
        respawn=True
    )
    ld.add_action(mavros_node3)

    return ld
