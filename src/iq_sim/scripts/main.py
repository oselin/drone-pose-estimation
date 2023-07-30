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
import Algorithms


def POSE_TOPIC_TEMPLATE(i):     return f"/drone{i}/mavros/local_position/pose"
def DISTANCE_TOPIC_TEMPLATE(i): return f"/drone{i}/mavros/distances"


TIMESTEP = 0.5

ANCHOR_MOVEMENT_TIME = 1.0  # 1 s

# SWARM_COEF = np.array([np.sqrt(2), np.sqrt(2), 0])
SWARM_COEF   = np.array([0.0, 1.0, 0.0])

ANCHOR_COEF  = np.vstack([np.eye(3), -np.eye(3)])

VELOCITY_MAGNITUDE = 1.0  # [m/s]


class Main(Node):

    def distance_reader_callback(self, received_msg, index):
        """
        Update the Distance Matrix (DM) buffer by replacing the i-th columnn.
        """
        self.DM_buffer[:, index] = np.array(received_msg.data)

    
    def move_swarm(self):
        """
        Move the drone swarm by sending velcity values.
        The method does not affect the anchor motion.
        """
        # Compute the velocity components
        vel_x, vel_y, vel_z = SWARM_COEF*VELOCITY_MAGNITUDE

        # Send velocity value
        for id in range(2, self.n_drones+1):
            self.navigation.send_setpoint_velocity(id, vel_x, vel_y, vel_z, 0.0)


    def move_anchor(self):
        """
        Move the anchor dron by sending velcity values.
        The method does not affect the swarm motion.
        """
        # Compute the velocity components
        vel_x, vel_y, vel_z = (SWARM_COEF + ANCHOR_COEF[self.phase_index]) * VELOCITY_MAGNITUDE

        # Send velocity value
        self.navigation.send_setpoint_velocity(self.anchor_id, vel_x, vel_y, vel_z, 0.0)


    def update(self):
        """
        Update the following class parameters:
            - timestamp
            - movement_time
            - phase_index
        """
        self.phase_index = (self.phase_index + 1) % len(ANCHOR_COEF)
        self.timestamp = time.time()
        self.movement_time = ANCHOR_MOVEMENT_TIME + Algorithms.noise(0, self.noise_time_std, shape=1)


    def read_distances(self):
        """
        Update the distances matrix read at phase phase_index and store it,
        as well as the anchor position. 
        """
        self.D_matrices[self.measurement_index] = self.DM_buffer
        self.P_matrices[:, self.measurement_index] = ANCHOR_COEF[self.phase_index] * \
            VELOCITY_MAGNITUDE * self.movement_time
        self.measurement_index = (self.measurement_index + 1) % 4


    def MDS(self):
        """
        Run MDS algorithm defined in the Algorithms class, by assembling the
        disance matrices in one unique [n+3, n+3] matrix.
        Return:
            - Coordinates of the drones swarm estimated via the algorithm.
        """
        # Assemble the full distance matrix
        DM = Algorithms.combine_matrices(
            self.D_matrices[0],    self.D_matrices[1],    self.D_matrices[2],    self.D_matrices[3],
            self.P_matrices[:, 0], self.P_matrices[:, 1], self.P_matrices[:, 2], self.P_matrices[:, 3]
            )

        return Algorithms.MDS(DM, self.P_matrices), None

    
    def WLP(self):
        """
        Run WLP algorithm defined in the Algorithms class, by assembling the
        disance matrices in one unique [n+3, n+3] matrix.
        Return:
            - Coordinates of the drones swarm estimated via the algorithm.
        """
        # Assemble the full distance matrix
        DM = Algorithms.combine_matrices(
            self.D_matrices[0],    self.D_matrices[1],    self.D_matrices[2],    self.D_matrices[3],
            self.P_matrices[:, 0], self.P_matrices[:, 1], self.P_matrices[:, 2], self.P_matrices[:, 3]
            )

        return Algorithms.WLP(DM, self.P_matrices), None


    def cycle_callback(self):
        """
        Node main loop.
        """
        # guide the swarm
        self.move_swarm()

        if ((time.time() - self.timestamp) >= self.movement_time):
            
            # Update distance matrices and anchors positions
            self.read_distances()

            # Run algorithms
            X_mds, Cov_mds = self.MDS()
            X_wlp, Cov_wlp = self.WLP()

            # update_plots()
            self.update()
            self.get_logger().info("Anchor moved; new phase index: %i" % self.phase_index)
        else:
            self.move_anchor()


    def initialize_swarm(self):
        """
        Initialize the swarm by appling the ArduCopter procedure.
            -1) Set mode to GUIDED
            -2) Arm the throttles
            -3) Take off to specified altitude
        """
        for id in range(1, self.n_drones+1): self.navigation.set_mode(id, "GUIDED")

        for id in range(1, self.n_drones+1):
            self.navigation.arm(id)
            self.navigation.takeoff(id, self.altitude)

        time.sleep(40.0)  # time to go up


    def __init__(self):
        super().__init__('main')
        print("Node that reads the distances, computes the coordinates, plots the results and guides the drones.")

        # Parameters from ROS2 command line
        self.declare_parameter('environment', 'gazebo')     # rclpy.Parameter.Type.STRING
        self.environment = self.get_parameter('environment').get_parameter_value().string_value
        
        self.declare_parameter('n_drones', 2)               # rclpy.Parameter.Type.INTEGER
        self.n_drones = self.get_parameter('n_drones').get_parameter_value().integer_value

        self.declare_parameter('altitude', 5.0)             # rclpy.Parameter.Type.DOUBLE
        self.altitude = self.get_parameter('altitude').get_parameter_value().double_value

        self.declare_parameter('noise_dist_std', 0.0)       # rclpy.Parameter.Type.DOUBLE
        self.noise_dist_std = self.get_parameter('noise_dist_std').get_parameter_value().double_value

        self.declare_parameter('noise_time_std', 0.0)       # rclpy.Parameter.Type.DOUBLE
        self.noise_time_std = self.get_parameter('noise_time_std').get_parameter_value().double_value

        
        # Attributes initialization
        self.phase_index, self.measurement_index = 0, 0
        self.anchor_id = 1
        self.movement_time = ANCHOR_MOVEMENT_TIME

        self.P_matrices = np.zeros((3, 4))
        self.D_matrices = np.zeros((4, self.n_drones, self.n_drones))
        self.DM_buffer  = np.zeros((self.n_drones, self.n_drones))

        # Subscribe to DISTANCE_TOPIC_TEMPLATE topic for each drone
        for i in range(self.n_drones):
            self.get_logger().info(f"Topic registered to {DISTANCE_TOPIC_TEMPLATE(i+1)} to read ")
            self.create_subscription(
                Float32MultiArray,
                DISTANCE_TOPIC_TEMPLATE(i+1),
                lambda msg, i=i: self.distance_reader_callback(msg, i),
                qos_profile_system_default
            )

        # Initialize Navigation object
        self.navigation = Navigation()

        if (self.environment == "gazebo"): self.initialize_swarm()

        # Initialize timer
        self.timestamp = time.time()

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
