#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import launch
from launch.actions import ExecuteProcess

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Find the path to the world file
    iq_sim_pkg = get_package_share_directory('iq_sim')

    print(iq_sim_pkg)
    world_file = iq_sim_pkg + '/worlds/multi_drone.world'

    # Create the launch description and specify the world file
    ld = launch.LaunchDescription()


    world_node = ExecuteProcess(
        cmd=[
            'gazebo',
            world_file
        ],
        output='screen' 
    )

    # Include the empty_world node in the launch description
    ld.add_action(world_node)

    return ld
