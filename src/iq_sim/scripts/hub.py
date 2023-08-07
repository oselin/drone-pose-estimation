#!/usr/bin/env python3

import numpy as np
import rclpy, Algorithms
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from Plot import class_name
from Control.topics import *



TIMESTEP = 0.1

class Hub(Node):

    def pose_reader_callback(self, received_msg, index):
        """
        Callaback function for the POSE_TOPIC_TEMPLATE topic.
        Save the information sent over the topic in the coords data structure.
        """
        pos = received_msg.pose.position
        self.coords[:, index] = (Algorithms.M_ROT_TRASL_DRONE_GZ(index) @ np.array([pos.x, pos.y, pos.z, 1]))[:3]


    def write_distances(self):
        """
        Publish distances on DISTANCE_TOPIC_TEMPLATE topic, as it was the UWB sensor's information.
        """
        for i in range(self.n_drones):
            msg = Float32MultiArray()
            msg.data = self.distances[:, i].tolist()
            self.distance_writers[i].publish(msg)

    def update_distances(self):
        noise =  Algorithms.noise(0, self.noise_dist_std, shape=self.distances.shape)
        self.distances = Algorithms.distance_matrix(self.coords) + noise ** 2

    def cycle_callback(self):
        """
        Cycle callback function. It works on a 2-steps approach.
            -1) Convert coordinates in distances
            -2) Send distances over the proper topic, as UWB sensor information
        """
        self.update_distances()
        self.write_distances()

    def __init__(self):

        # Declare the ROS2 node
        class_name(self)
        super().__init__('hub')
        self.get_logger().info("Node to read the coordinates and write the distances")

        # Parameters from ROS2 command line
        self.declare_parameter('n_drones', rclpy.Parameter.Type.INTEGER)
        self.n_drones = self.get_parameter('n_drones').get_parameter_value().integer_value

        # rclpy.Parameter.Type.DOUBLE
        self.declare_parameter('noise_dist_std', rclpy.Parameter.Type.DOUBLE)
        self.noise_dist_std = self.get_parameter(
            'noise_dist_std').get_parameter_value().double_value

        # Pre-allocation of memory
        self.distances = np.zeros((self.n_drones, self.n_drones))         
        self.coords = np.zeros((3, self.n_drones))   

        # Subscribe to POSE_TOPIC_TEMPLATE topic for each drone
        for i in range(self.n_drones):
            self.get_logger().info(f"Read from {POSE_TOPIC_TEMPLATE(i+1)}")
            self.create_subscription(
                PoseStamped,
                POSE_TOPIC_TEMPLATE(i+1),
                lambda msg, i=i: self.pose_reader_callback(msg, i),
                qos_profile_system_default
            )

        # Publish to DISTANCE_TOPIC_TEMPLATE topic for each drone and save i-th client to list
        self.distance_writers = []
        for i in range(self.n_drones):
            self.get_logger().info(f"Write to {DISTANCE_TOPIC_TEMPLATE(i+1)}")
            self.distance_writers.append(self.create_publisher(
                Float32MultiArray,
                DISTANCE_TOPIC_TEMPLATE(i+1),
                qos_profile_system_default
            ))

        # Loop over time
        self.timer = self.create_timer(TIMESTEP, self.cycle_callback)


def main(args=None):
    rclpy.init(args=args)
    hub = Hub()
    rclpy.spin(hub)

    hub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
