#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node
import rclpy

import numpy as np

from Algorithms import *
from Plot import class_name

def POSE_TOPIC_TEMPLATE(i):     return f"/drone{i}/mavros/local_position/pose"
def VELOCITY_TOPIC_TEMPLATE(i): return f"/drone{i}/mavros/setpoint_velocity/cmd_vel"
def M_ROT_TRASL_Z_GZ_DRONE(i): return np.array([[0,-1,0,0], [1,0,0,-(i+1)], [0,0,1,0], [0,0,0,1]])
# M_ROT_TRASL_Z_GZ_DRONE = MatrixInverse(M_ROT_TRASL_Z_DRONE_GZ)

TIMESTEP = 0.1 # to put in the config file

class Test(Node):

    def velocity_reader_callback(self, received_msg, index):
        """
        Function to handle incoming messages from VELOCITY_TOPIC_TEMPLATE topics
        Parameters:
            - received_msg: message coming from topic
            - index: drone index generated at subscription time
        """
        vel = received_msg.twist.linear
        self.states[3:, index] = np.array([vel.x, vel.y, vel.z])
        self.get_logger().info(f"New velocity for drone{index+1} received")

    def update_positions(self):
        """
        Integrator of first order: s = v * t
        states variable stores information as following: [x, y, z, vel_x, vel_y, vel_z]
        """
        self.states[:3] += self.states[3:] * TIMESTEP


    def write_positions(self):
        """
        Send updated position via ROS2 topics
        The transformations simulate the APM conventions (rotation of pi about x)
        """
        for i in range(self.n_drones):
            pos = M_ROT_TRASL_Z_GZ_DRONE(i) @ np.hstack((self.states[:3,i], 1))

            pose_msg = PoseStamped()
            pose_msg.pose.position.x = pos[0]
            pose_msg.pose.position.y = pos[1]
            pose_msg.pose.position.z = pos[2]

            self.writers[i].publish(pose_msg)


    def cycle_callback(self):
        """
        Callback function that allows to run over time.
        It works on a 2-steps approach.
            -1 step: update drones position
            -2 step: write the updated position as PoseStamped message
        """
        self.update_positions()
        self.write_positions()

        self.get_logger().info("Positions updated")
        self.get_logger().info(str(self.states))


    def __init__(self):

        # Declare the ROS2 node
        class_name(self)
        super().__init__('test')
        self.get_logger().info("Node that simulates the simulator.")

        # Parameters from ros2 command line
        self.declare_parameter('n_drones', rclpy.Parameter.Type.INTEGER)
        self.n_drones = self.get_parameter('n_drones').get_parameter_value().integer_value

        self.declare_parameter('altitude', rclpy.Parameter.Type.DOUBLE)
        self.altitude = self.get_parameter('altitude').get_parameter_value().double_value

        # Class attributes, initialized for allocating memory
        self.states    = np.zeros((6, self.n_drones))               # size = 6: x, y, z, vel_x, vel_y, vel_z
        self.states[0] = np.array([range(1, self.n_drones+1)])
        self.states[2] = np.tile(self.altitude, (1, self.n_drones)) # set z value to the one provided
        self.writers   = np.tile(None, (self.n_drones, ))

        # Subscribe to VELOCITY_TOPIC_TEMPLATE topic for each drone
        for i in range(self.n_drones):
            self.get_logger().info(f"Topic registered to {VELOCITY_TOPIC_TEMPLATE(i+1)} to read ")
            self.create_subscription(
                TwistStamped,
                VELOCITY_TOPIC_TEMPLATE(i+1),
                lambda msg, i=i: self.velocity_reader_callback(msg, i),
                qos_profile_system_default
            )

        # Publish to POSE_TOPIC_TEMPLATE topic for each drone
        for i in range(self.n_drones):
            self.get_logger().info(f"Topic registered to {POSE_TOPIC_TEMPLATE(i+1)} to write ")
            self.writers[i] = self.create_publisher(
                PoseStamped,
                POSE_TOPIC_TEMPLATE(i+1),
                qos_profile_system_default
            )

        # Loop over time
        self.timer = self.create_timer(TIMESTEP, self.cycle_callback)


def main(args=None):
    rclpy.init(args=args)
    test = Test()
    rclpy.spin(test)

    test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

