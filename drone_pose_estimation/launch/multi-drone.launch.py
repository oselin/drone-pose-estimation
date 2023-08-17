#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import launch
from launch.actions import ExecuteProcess

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Find the path to the world file
    drone_pose_estimation_pkg = get_package_share_directory('drone_pose_estimation')

    print(drone_pose_estimation_pkg)
    world_file = drone_pose_estimation_pkg + '/worlds/multi_drone.world'

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
