#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node
import rclpy

import numpy as np

from Algorithms import *
from Plot import class_name


def POSE_TOPIC_TEMPLATE(id): return f"/drone{id}/mavros/local_position/pose"


def VEL_TOPIC_TEMPLATE(
    id): return f"/drone{id}/mavros/setpoint_velocity/cmd_vel"


def M_ROT_TRASL_GZ_DRONE(i): return np.array(
    [[0, -1, 0, 0], [1, 0, 0, -i], [0, 0, 1, 0], [0, 0, 0, 1]])
def M_ROT_TRASL_GZ_DRONE(i): return np.eye(4)

# M_ROT_TRASL_GZ_DRONE = MatrixInverse(M_ROT_TRASL_Z_DRONE_GZ)


TIMESTEP = 0.02 # to put in the config file # 10 ms are enough


class Test(Node):

    def velocity_reader_callback(self, received_msg, index):
        """
        Function to handle incoming messages from VEL_TOPIC_TEMPLATE topics
        Parameters:
            - received_msg: message coming from topic
            - index: drone index generated at subscription time
        """
        vel = received_msg.twist.linear
        self.states[3:, index] = np.array([vel.x, vel.y, vel.z])
        self.get_logger().debug(f"New velocity drone{index+1}: {str(vel)}")

    def update(self):
        """
        Integrator of first order: s = v * t
        states variable stores information as following: [x, y, z, vel_x, vel_y, vel_z]
        """
        now_timestamp = self.get_timestamp()
        self.states[:3] += self.states[3:] * (now_timestamp - self.timestamp)
        self.timestamp = now_timestamp

    def write(self):
        """
        Send updated position via ROS2 topics
        The transformations simulate the APM conventions (rotation of pi about x)
        """
        for i in range(self.n_drones):
            pos = M_ROT_TRASL_GZ_DRONE(
                i) @ np.hstack((self.states[:3, i], 1))

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
        self.update()
        self.write()

    def __init__(self):

        # Declare the ROS2 node
        class_name(self)
        super().__init__('test')
        self.get_logger().info("Node that simulates the simulator.")

        # Parameters from ros2 command line
        self.declare_parameter('n_drones', rclpy.Parameter.Type.INTEGER)
        self.n_drones = self.get_parameter(
            'n_drones').get_parameter_value().integer_value

        # Class attributes, initialized for allocating memory
        # states = [[x, y, z, vel_x, vel_y, vel_z]^T, ...]
        self.states = np.zeros((6, self.n_drones))
        self.states[:3, 0] = np.array([-2,0,0])
        self.states[0] = np.array([range(self.n_drones)])
        self.writers = np.tile(None, (self.n_drones, ))

        def get_timestamp():
            now = self.get_clock().now().to_msg()
            return now.sec+now.nanosec/1e9
        self.get_timestamp = get_timestamp
        
        self.timestamp = self.get_timestamp()

        # Subscribe to VEL_TOPIC_TEMPLATE topic for each drone
        for i in range(self.n_drones):
            self.get_logger().info(f"Read from {VEL_TOPIC_TEMPLATE(i+1)}")
            self.create_subscription(
                TwistStamped,
                VEL_TOPIC_TEMPLATE(i+1),
                lambda msg, i=i: self.velocity_reader_callback(msg, i),
                qos_profile_system_default
            )

        # Publish to POSE_TOPIC_TEMPLATE topic for each drone
        for i in range(self.n_drones):
            self.get_logger().info(f"Write to {POSE_TOPIC_TEMPLATE(i+1)}")
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
