#!/usr/bin/env python3

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node
import rclpy

import numpy as np
import time

from Control import Navigation
import Algorithms
from Plot import class_name, Plot


def POSE_TOPIC_TEMPLATE(id): return f"/drone{id}/mavros/local_position/pose"
def DISTANCE_TOPIC_TEMPLATE(id): return f"/drone{id}/mavros/distances"


TIMESTEP = 0.05
ANCHOR_MOV_TIME = 1  # 1 s

SWARM_COEF = np.array([0.0, 1.0, 0.0])
ANCHOR_COEF = np.vstack([-np.eye(3),np.eye(3)])

SWARM_VEL = 0.0  # [m/s]
ANCHOR_VEL = 0.5


def M_ROT_TRASL_DRONE_GZ(i): return np.array(
    [[0, 1, 0, i], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


class Main(Node):

    def pose_reader_callback(self, received_msg, index):
        """
        Callback function for the POSE_TOPIC_TEMPLATE topic.
        Save the information sent over the topic in the coords data structure.
        It is activated only if 'environment' is set to 'test'
        """
        pos = received_msg.pose.position
        self.coords[:, index] = (M_ROT_TRASL_DRONE_GZ(
            index) @ np.array([pos.x, pos.y, pos.z, 1]))[:3]
    
    def distance_reader_callback(self, received_msg, index):
        """
        Update the Distance Matrix (DM) buffer by replacing the i-th columnn.
        """
        self.DM_buffer[:, index] = np.array(received_msg.data)

    def initialize_swarm(self):
        """
        Initialize the swarm by appling the ArduCopter procedure.
            -1) Set mode to GUIDED
            -2) Arm the throttles
            -3) Take off to specified altitude
        """
        for id in range(1, self.n_drones+1):
            self.navigation.set_mode(id, "GUIDED")

        for id in range(1, self.n_drones+1):
            self.navigation.arm(id)
            self.navigation.takeoff(id, self.altitude)

        time.sleep(40.0)  # time to go up

    def update(self, new_timestamp):
        """
        Update the following class parameters:
            - anchor position
            - distance matrix
            - measurement index
            - phase index
            - offset
            - movement time
            - timestamp
        """
        # practically
        self.PMs[:, self.meas_index] = self.PMs[:, self.meas_index -
                                                1] + ANCHOR_COEF[self.phase_index] * ANCHOR_VEL * self.mov_time
        self.DMs[self.meas_index] = np.copy(self.DM_buffer)

        # ideally
        # # # self.PMs[:, self.meas_index] = self.coords[:, 0]
        # # # self.DMs[self.meas_index] = Algorithms.distance_matrix(self.coords)

        self.meas_index = (self.meas_index + 1) % self.n_meas
        self.phase_index = (self.phase_index + 1) % len(ANCHOR_COEF)

        # print("Time difference:")
        # print(new_timestamp-self.timestamp)
        noise_time = Algorithms.noise(0, self.noise_time_std, shape=1)
        self.mov_time = ANCHOR_MOV_TIME + noise_time
        self.offset = np.copy(self.coords[:, 0])
        self.timestamp = new_timestamp

        if (not self.algorithms and self.phase_index > 3):
            self.algorithms = True

    def MDS(self):
        """
        Run MDS algorithm defined in the Algorithms class, by assembling the
        disance matrices in one unique [n+3, n+3] matrix.
        Return:
            - Coordinates of the drones swarm estimated via the algorithm.
        """
        # Assemble the full distance matrix
        DM = Algorithms.combine_matrices(
            self.DMs[0], self.DMs[1], self.DMs[2], self.DMs[3],
            self.PMs[:, 0], self.PMs[:, 1], self.PMs[:, 2], self.PMs[:, 3]
        )

        return Algorithms.MDS(DM, self.PMs), None

    def WLP(self):
        """
        Run WLP algorithm defined in the Algorithms class, by assembling the
        disance matrices in one unique [n+3, n+3] matrix.
        Return:
            - Coordinates of the drones swarm estimated via the algorithm.
        """
        # Assemble the full distance matrix
        DM = Algorithms.combine_matrices(
            self.DMs[0], self.DMs[1], self.DMs[2], self.DMs[3],
            self.PMs[:, 0], self.PMs[:, 1], self.PMs[:, 2], self.PMs[:, 3]
        )

        return Algorithms.WLP(DM, self.PMs), None

    def move_swarm(self):
        """
        Move the drone swarm by sending velcity values.
        The method does not affect the anchor motion.
        """
        # Compute the velocity components
        vel_x, vel_y, vel_z = SWARM_COEF*SWARM_VEL

        # Send velocity value
        for id in range(2, self.n_drones+1):
            self.navigation.send_setpoint_velocity(
                id, vel_x, vel_y, vel_z, 0.0)

    def move_anchor(self):
        """
        Move the anchor dron by sending velcity values.
        The method does not affect the swarm motion.
        """
        # Compute the velocity components
        vel_x, vel_y, vel_z = SWARM_COEF * SWARM_VEL + \
            ANCHOR_COEF[self.phase_index] * ANCHOR_VEL

        # Send velocity value
        self.navigation.send_setpoint_velocity(
            self.anchor_id, vel_x, vel_y, vel_z, 0.0)

    def cycle_callback(self):
        """
        Node main loop.
        """
        # guide the swarm

        now_timestamp = self.get_timestamp()
        if ((now_timestamp - self.timestamp) >= self.mov_time):
            # Update distance matrices, anchors positions and indicicies
            self.update(new_timestamp=now_timestamp)

            # Run algorithms
            if (self.algorithms):
                X_mds, Cov_mds = self.MDS()
                X_wlp, Cov_wlp = self.WLP()

                self.coords[:, 0].reshape(-1, 1)
                self.plot.update(
                    true_coords=np.copy(self.coords),
                    MDS_coords=X_mds,  # + self.offset.reshape(-1, 1),
                    WLP_coords=X_wlp,  # + self.offset.reshape(-1, 1),
                    MDS_cov=None,
                    WLP_cov=None
                )

            self.get_logger().info(f"New phase index: {self.phase_index}")
            self.get_logger().info(f"  Anchor moved: {str(self.coords[:, 0])}")

        self.move_swarm()
        self.move_anchor()
        # print("sec")
        # print(a+self.get_clock().now().to_msg().sec+self.get_clock().now().to_msg().nanosec/1e9)

    def __init__(self):

        # Declare the ROS2 node
        class_name(self)
        super().__init__('main')
        print("Node that reads the distances, computes the coordinates, plots the results and guides the drones.")

        # Parameters from ROS2 command line
        # rclpy.Parameter.Type.STRING
        self.declare_parameter('environment', 'gazebo')
        self.environment = self.get_parameter(
            'environment').get_parameter_value().string_value

        # rclpy.Parameter.Type.INTEGER
        self.declare_parameter('n_drones', 2)
        self.n_drones = self.get_parameter(
            'n_drones').get_parameter_value().integer_value

        # rclpy.Parameter.Type.DOUBLE
        self.declare_parameter('altitude', 5.0)
        self.altitude = self.get_parameter(
            'altitude').get_parameter_value().double_value

        # rclpy.Parameter.Type.DOUBLE
        self.declare_parameter('noise_dist_std', 0.0)
        self.noise_dist_std = self.get_parameter(
            'noise_dist_std').get_parameter_value().double_value

        # rclpy.Parameter.Type.DOUBLE
        self.declare_parameter('noise_time_std', 0.0)
        self.noise_time_std = self.get_parameter(
            'noise_time_std').get_parameter_value().double_value

        # Attributes initialization
        self.phase_index, self.meas_index = 0, 0
        self.anchor_id = 1
        self.n_meas = 4
        self.mov_time = ANCHOR_MOV_TIME
        self.algorithms = False

        self.coords = np.zeros((3, self.n_drones))
        self.offset = np.zeros((3,))
        self.PMs = np.zeros((3, self.n_meas))
        self.DMs = np.zeros((self.n_meas, self.n_drones, self.n_drones))
        self.DM_buffer = np.zeros((self.n_drones, self.n_drones))

        # Subscribe to DISTANCE_TOPIC_TEMPLATE topic for each drone
        for i in range(self.n_drones):
            self.get_logger().info(
                f"Read from {DISTANCE_TOPIC_TEMPLATE(i+1)}")
            self.create_subscription(
                Float32MultiArray,
                DISTANCE_TOPIC_TEMPLATE(i+1),
                lambda msg, i=i: self.distance_reader_callback(msg, i),
                qos_profile_system_default
            )

        # Subscribe to POSE_TOPIC_TEMPLATE topic for each drone
        for i in range(self.n_drones):
            self.get_logger().info(
                f"Read from {POSE_TOPIC_TEMPLATE(i+1)}")
            self.create_subscription(
                PoseStamped,
                POSE_TOPIC_TEMPLATE(i+1),
                lambda msg, i=i: self.pose_reader_callback(msg, i),
                qos_profile_system_default
            )

        # Initialize Navigation object
        self.navigation = Navigation(
            node=self, n_drones=self.n_drones, timeout=10)
        if (self.environment == "gazebo"):
            self.initialize_swarm()

        # Initialize Plot object
        self.plot = Plot(mode='2D', display_MDS=True,
                         display_WLP=True, reduction_method='xy')

        # Timestamps
        def get_timestamp():
            now = self.get_clock().now().to_msg()
            return now.sec+now.nanosec/1e9
        self.get_timestamp = get_timestamp

        self.timestamp = self.get_timestamp()

        self.step = 0
        # Start the plot thread
        self.plot.start()

        # Loop over time
        self.timer = self.create_timer(TIMESTEP, self.cycle_callback)


def main(args=None):
    rclpy.init(args=args)
    main = Main()
    rclpy.spin(main)

    main.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
