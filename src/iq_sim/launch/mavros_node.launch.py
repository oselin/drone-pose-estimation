from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    fcu_url = DeclareLaunchArgument('fcu_url', default_value='')
    gcs_url = DeclareLaunchArgument('gcs_url', default_value='')
    mavros_ns = DeclareLaunchArgument('mavros_ns', default_value='')
    tgt_system = DeclareLaunchArgument('tgt_system')
    tgt_component = DeclareLaunchArgument('tgt_component')
    pluginlists_yaml = DeclareLaunchArgument('pluginlists_yaml')
    config_yaml = DeclareLaunchArgument('config_yaml')
    log_output = DeclareLaunchArgument('log_output', default_value='screen')
    fcu_protocol = DeclareLaunchArgument('fcu_protocol', default_value='v2.0')
    respawn_mavros = DeclareLaunchArgument('respawn_mavros', default_value='false')

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        namespace=LaunchConfiguration('mavros_ns'),
        output=LaunchConfiguration('log_output'),
        parameters=[
            {'fcu_url': LaunchConfiguration('fcu_url')},
            {'gcs_url': LaunchConfiguration('gcs_url')},
            {'target_system_id': LaunchConfiguration('tgt_system')},
            {'target_component_id': LaunchConfiguration('tgt_component')},
            {'fcu_protocol': LaunchConfiguration('fcu_protocol')}
        ],
        remappings=[
            ('~/pluginlists.yaml', LaunchConfiguration('pluginlists_yaml')),
            ('~/config.yaml', LaunchConfiguration('config_yaml'))
        ],
        emulate_tty=True,
        respawn=LaunchConfiguration('respawn_mavros')
    )

    return LaunchDescription([
        fcu_url,
        gcs_url,
        mavros_ns,
        tgt_system,
        tgt_component,
        pluginlists_yaml,
        config_yaml,
        log_output,
        fcu_protocol,
        respawn_mavros,
        mavros_node
    ])

