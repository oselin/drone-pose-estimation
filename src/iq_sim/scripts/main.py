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
def DISTANCE_TOPIC_TEMPLATE(i): return f"/drone{i}/mavros/distances"


TIMESTEP = 0.5

ANCHOR_MOVEMENT_TIME = 1.0  # 1 s

# SWARM_COEF = np.array([np.sqrt(2), np.sqrt(2), 0])
SWARM_COEF = np.array([0.0, 1.0, 0.0])

ANCHOR_COEF = np.array([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
    [-1, 0, 0],
    [0, -1, 0],
    [0, 0, -1],
])

VELOCITY_MAGNITUDE = 1.0  # [m/s]


class Main(Node):
    def distance_reader_callback(self, received_msg, index):
        self.distances[:, index] = np.array(received_msg.data)
        # print('Received distances for drone%i: %s' % (index+1, str(self.distances[:, index])))

    def leggi e mds():
        # logica per gestire il movimento
        # calcolo distanza integrata
        # Richiamo funzioni ALgebra.py

        # se passo moviment/direzione alora CHiamo MDS etc di algebra

        # se no nuovo algorimto con Exp-Min

    def move_swarm(self):
        vel_x, vel_y, vel_z = SWARM_COEF*VELOCITY_MAGNITUDE
        for id in range(2, self.n_drones+1):
            self.navigation.send_setpoint_velocity(
                id, vel_x, vel_y, vel_z, 0.0)

    def move_anchor(self):
        # move the anchor following the specified mode and collect new distances after ANCHOR_MOVEMENT_TIME
        vel = (SWARM_COEF + ANCHOR_COEF[self.phase_index]) * VELOCITY_MAGNITUDE
        vel_x, vel_y, vel_z = vel

        self.navigation.send_setpoint_velocity(
            self.anchor_id, vel_x, vel_y, vel_z, 0.0)

    def update_times(self):
        self.timestamp = time.time()
        self.movement_time = ANCHOR_MOVEMENT_TIME + \
            noise(0, self.noise_time_std,  1)

    def read_distances(self):
        self.D_matrices[self.measurement_index] = self.distances
        self.P_matrices[:, self.measurement_index] = ANCHOR_COEF[self.phase_index] * \
            VELOCITY_MAGNITUDE * self.movement_time
        self.measurement_index = (self.measurement_index + 1) % 4

    def run_MDS(self):
        DM = combine_matrices(
            self.D_matrices[0],
            self.D_matrices[1],
            self.D_matrices[2],
            self.D_matrices[3],
            self.P_matrices[:, 0],
            self.P_matrices[:, 1],
            self.P_matrices[:, 2],
            self.P_matrices[:, 3]
        )
        MDS(DM, self.P_matrices)

    def cycle_callback(self):
        # guide the swarm
        self.move_swarm()
        if time.time() >= self.timestamp + self.movement_time:
            # create DM
            self.read_distances()
            self.run_MDS()

            # run MDS(movimento)

            # update_plots()
            self.update_times()
            self.phase_index = (self.phase_index + 1) % len(ANCHOR_COEF)
            self.get_logger().info("Anchor moved; new phase index: %i" % self.phase_index)
        else:
            self.move_anchor()

    def initialize_swarm(self):
        for id in range(1, self.n_drones+1):
            self.navigation.set_mode(id, "GUIDED")

        for id in range(1, self.n_drones+1):
            self.navigation.arm(id)
            self.get_logger().info(f'[drone{id}] Armed.')
            self.navigation.takeoff(id, 5.0)

        time.sleep(40.0)  # time to go up

    def __init__(self):
        super().__init__('main')
        print("Node that reads the distances, computes the coordinates, plots the results and guides the drones.")

        # ROS parameters
        # rclpy.Parameter.Type.STRING)
        self.declare_parameter('environment', 'gazebo')
        # rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('n_drones', 2)
        # rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('altitude', 5.0)
        # rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('noise_dist_std', 0.0)
        # rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('noise_time_std', 0.0)

        # Class attributes
        # received
        self.environment = self.get_parameter(
            'environment').get_parameter_value().string_value
        self.n_drones = self.get_parameter(
            'n_drones').get_parameter_value().integer_value
        self.altitude = self.get_parameter(
            'altitude').get_parameter_value().double_value
        self.noise_dist_std = self.get_parameter(
            'noise_dist_std').get_parameter_value().double_value
        self.noise_time_std = self.get_parameter(
            'noise_time_std').get_parameter_value().double_value

        self.distances = np.ones((self.n_drones, self.n_drones))

        # measured
        self.measurement_index = 0
        self.D_matrices = np.zeros((4, self.n_drones, self.n_drones))
        self.P_matrices = np.zeros((3, 4))

        # calculated
        self.DM = np.zeros((self.n_drones+3, self.n_drones+3))
        self.MDS_coords = np.ones(
            (3, self.n_drones))
        self.WLP_coords = np.ones(
            (3, self.n_drones))
        self.MDS_cov = np.ones((9, self.n_drones))

        # Topics
        # read the array of distances for each drone separatedly
        for i in range(self.n_drones):
            self.get_logger().info(
                f"Topic registered to {DISTANCE_TOPIC_TEMPLATE(i+1)} to read ")
            self.create_subscription(
                Float32MultiArray,
                DISTANCE_TOPIC_TEMPLATE(i+1),
                lambda msg, i=i: self.distance_reader_callback(msg, i),
                qos_profile_system_default
            )

        # Initialize navigation
        self.navigation = Navigation()

        if self.environment == "gazebo":
            self.initialize_swarm()

        # Start cycle
        self.anchor_id = 1
        self.phase_index = 0        # for managing the execution
        self.movement_time = ANCHOR_MOVEMENT_TIME
        self.timestamp = time.time()
        self.timer = self.create_timer(TIMESTEP, self.cycle_callback)


def main(args=None):
    rclpy.init(args=args)
    main = Main()
    rclpy.spin(main)

    main.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
