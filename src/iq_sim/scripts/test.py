#!/usr/bin/env python3
from std_msgs.msg import Float32MultiArray
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node
import rclpy

import numpy as np
import time

from Control import Navigation
from Algorithms import *

def POSE_TOPIC_TEMPLATE(i): return f"/drone{i}/mavros/local_position/pose"
def VELOCITY_TOPIC_TEMPLATE(i): return f"/drone{i}/mavros/setpoint_velocity/cmd_vel"


TIMESTEP = 0.1

class Test(Node):
    def velocity_reader_callback(self, received_msg, index):
        vel = received_msg.twist.linear
        self.states[3:, index] = np.array([vel.x, vel.y, vel.z])
        self.get_logger().info(f"New velocity for drone{index} received")


    def update_positions(self):
        self.states[:3] += self.states[3:] * TIMESTEP

    def write_positions(self):
        for i in range(self.n_drones):
            pose_msg = PoseStamped()
            pose_msg.pose.position.x = self.states[0, i] 
            pose_msg.pose.position.y = self.states[1, i] 
            pose_msg.pose.position.z = self.states[2, i] 
            
            self.writers[i].publish(pose_msg)

    def cycle_callback(self):
        self.update_positions()
        self.write_positions()

        self.get_logger().debug("Positions updated")

    def __init__(self):
        super().__init__('test')
        self.get_logger().info("Node that simulates the simulator.")

        # ROS parameters
        self.declare_parameter('n_drones', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('altitude', rclpy.Parameter.Type.DOUBLE)

        self.n_drones = self.get_parameter(
            'n_drones').get_parameter_value().integer_value
        
        self.altitude = self.get_parameter(
            'altitude').get_parameter_value().double_value

        self.states = np.zeros((6, self.n_drones))
        self.states[2, :] = np.tile(self.altitude, (1, self.n_drones))
        self.writers = np.tile(None, (self.n_drones, ))

        # Topics
        # read the array of distances for each drone separatedly
        for i in range(self.n_drones):
            print(f"Topic registered to {VELOCITY_TOPIC_TEMPLATE(i+1)} to read ")
            self.create_subscription(
                TwistStamped,
                VELOCITY_TOPIC_TEMPLATE(i+1),
                lambda msg, i=i: self.velocity_reader_callback(msg, i),
                qos_profile_system_default
            )

        # write the positions of each drone separatedly
        for i in range(self.n_drones):
            print(f"Topic registered to {POSE_TOPIC_TEMPLATE(i+1)} to write ")
            self.writers[i] = self.create_publisher(
                PoseStamped,
                POSE_TOPIC_TEMPLATE(i+1),
                qos_profile_system_default
            )

        self.timer = self.create_timer(TIMESTEP, self.cycle_callback)


def main(args=None):
    rclpy.init(args=args)
    test = Test()
    rclpy.spin(test)

    test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# def simulation(parameters):
    
#     # Initialize UAVs coordinates, randomly
#     X = np.random.uniform(low = -5, high=5, size=[3, parameters['number_uavs']])

#     alpha = 1
#     mean, sigma = 0, 0.02

#     while True:

#         #
#         # ANCHOR - position of the anchor, after the applied motion
#         # X      - coordinates of the fleet
#         #

#         # Retrieve the distances and build the distance matrix DM. In reality it comes from UWB sensors
#         ANCHOR1, X = move_anchor(points = X, step = 0)
#         DM1  = distance_matrix(X) + noise(mean=mean, std=sigma, shape=parameters['number_uavs'])       

#         # Simulate a second virtual anchor, by moving the real one and retrieving distances
#         ANCHOR2, X  = move_anchor(points = X, step = 1, displacement=alpha)
#         DM2 = distance_matrix(X) + noise(mean=mean, std=sigma, shape=parameters['number_uavs'])       
        
#         # Simulate a third virtual anchor, by moving the real one and retrieving distances
#         ANCHOR3, X  = move_anchor(points = X, step = 2, displacement=alpha)
#         DM3 = distance_matrix(X) + noise(mean=mean, std=sigma, shape=parameters['number_uavs'])       
        
#         # Simulate a fourth virtual anchor, by moving the real one and retrieving distances
#         ANCHOR4, X  = move_anchor(points = X, step = 3, displacement=alpha)
#         DM4 = distance_matrix(X) + noise(mean=mean, std=sigma, shape=parameters['number_uavs'])       
        
#         # Assemble the distance information in one unique matrix
#         DM = combine_matrices(DM1, DM2, DM3, DM4, ANCHOR1, ANCHOR2, ANCHOR3, ANCHOR4)

#         # Store the anchor and virtual anchors position into a coordinates array
#         anchor_pos = np.hstack([ANCHOR1, ANCHOR2, ANCHOR3, ANCHOR4])

#         # Return to the initial point
#         _, X = move_anchor(points = X, step = 4, displacement=alpha)

#         # Estimate the fleet coordinates
#         X_hat = MDS(DM, anchor_pos)

#         # Plot the scenario
#         plot_uavs(true_coords=X, estimated_coords=X_hat)
        
#         # Make the fleet move, except for the anchor
#         X = move_fleet(points = X, low = -2, high = 2)
