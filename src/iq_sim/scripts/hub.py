#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import SetMode, CommandBool
import tf2_ros

from std_msgs.msg import Float32MultiArray

POSE_TOPIC_TEMPLATE = lambda i : f"/drone{i}/local_position/pose"
DISTANCE_TOPIC_TEMPLATE = lambda i : f"/drone{i}/distances"
TIMESTEP = 0.5

class Hub(Node):
    def write_distances(self):
        for i in range(self.n_drones):
            msg = Float32MultiArray()
            msg.data = self.distances[:, i].tolist()
            self.distance_writers[i].publish(msg)
            #self.get_logger().info('Distances for drone%i: %s' % (i+1, str(msg.data)))

    def pose_reader_callback(self, received_msg, index):
        pos = received_msg.pose.position
        #self.get_logger().info('Received pose from drone%i: %s' % (index+1, str(received_msg)))
        self.coords[:, index] = [pos.x + index + 1, pos.y, pos.z]
        print(f"Drone {index+1}:",self.coords[:,index])

            
    def compute_distances(self):
        X = self.coords
        e = np.ones(X.shape[1]).reshape(-1,1)
        Phi = np.diag(X.T @ X).reshape(-1,1)
        D = Phi @ e.T - 2* X.T @ X + e @ Phi.T
        self.distances = D

    def cycle_callback(self):
        self.compute_distances()
        self.write_distances()


    def __init__(self):
        super().__init__('hub')
        self.get_logger().info("Node to read the coordinates and write the distances")

        ## ROS parameters
        self.declare_parameter('n_drones', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('noise',    rclpy.Parameter.Type.STRING)

        # Class attributes
        self.n_drones = self.get_parameter('n_drones').get_parameter_value().integer_value
        self.noise    = self.get_parameter('noise').get_parameter_value().string_value

        self.distances = np.ones((self.n_drones, self.n_drones))
        self.coords    = np.ones((3, self.n_drones))

        # Create tf2 Transformation instances
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_lister = tf2_ros.TransformListener(self.tf_buffer, self)
        # Subscribe to the topic
        for i in range(self.n_drones):
            print("Topic registered to %s " %str(POSE_TOPIC_TEMPLATE(i+1)))
            self.create_subscription(
                PoseStamped,
                POSE_TOPIC_TEMPLATE(i+1),
                lambda msg, i=i: self.pose_reader_callback(msg, i),
                qos_profile_system_default
            )

        # write the array of distances for each drone separatedly
        self.distance_writers = []
        for i in range(self.n_drones):
            print("Topic registered to %s to write " %str(DISTANCE_TOPIC_TEMPLATE(i+1)))
            self.distance_writers.append(self.create_publisher(
                Float32MultiArray,
                DISTANCE_TOPIC_TEMPLATE(i+1),
                qos_profile_system_default
            ))

        ## start cycle
        self.timer = self.create_timer(TIMESTEP, self.cycle_callback)

def main(args=None):
    rclpy.init(args=args)
    hub = Hub()
    rclpy.spin(hub)

    hub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()