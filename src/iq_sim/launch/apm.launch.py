from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

# ros2 launch iq_sim apm.launch.py fcu_url:=udp://127.0.0.1:14561@14565 mavros_ns:=/drone2 tgt_system:=2

def generate_launch_description():
    fcu_url = LaunchConfiguration('fcu_url', default='udp://127.0.0.1:14551@14555')
    gcs_url = LaunchConfiguration('gcs_url', default='')
    tgt_system = LaunchConfiguration('tgt_system', default='1')
    tgt_component = LaunchConfiguration('tgt_component', default='1')
    log_output = LaunchConfiguration('log_output', default='screen')
    respawn_mavros = LaunchConfiguration('respawn_mavros', default='true')
    mavros_ns = LaunchConfiguration('mavros_ns', default='/')
    config_yaml = LaunchConfiguration('config_yaml', default=[get_package_share_directory('mavros'), '/launch/apm_config.yaml'])

    mavros_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('iq_sim'), '/launch/mavros_node.launch.py']),
        launch_arguments={
            'pluginlists_yaml': [get_package_share_directory('mavros'), '/launch/apm_pluginlists.yaml'],
            'config_yaml': config_yaml,
            'mavros_ns': mavros_ns,
            'fcu_url': fcu_url,
            'gcs_url': gcs_url,
            'respawn_mavros': respawn_mavros,
            'tgt_system': tgt_system,
            'tgt_component': tgt_component,
            'log_output': log_output
        }.items()
    )

    return LaunchDescription([mavros_node_launch])
