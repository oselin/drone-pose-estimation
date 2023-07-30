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


# DA TRASFORMARE IN PARAMETRI:
noise_std = 0.2


class Main(Node):
    def distance_reader_callback(self, received_msg, index):
        self.distances[:, index] = np.array(received_msg.data)
        # print('Received distances for drone%i: %s' % (index+1, str(self.distances[:, index])))

    # def leggi e mds():
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
        if self.mode == 2:
            self.movement_time = ANCHOR_MOVEMENT_TIME + \
                noise(0, noise_std,  [1, 0])

    def cycle_callback(self):
        # guide the swarm
        self.move_swarm()
        if time.time() >= self.timestamp + self.movement_time:
            # leggi e MDS(movimento)
            # update_plots()
            self.update_times()
            self.phase_index = (self.phase_index + 1) % len(ANCHOR_COEF)
            # print("Anchor moved; new phase index: %i" % self.phase_index)
        else:
            self.move_anchor()

    def initialize_swarm(self):
        for id in range(1, self.n_drones+1):
            self.navigation.set_mode(id, "GUIDED")

        for id in range(1, self.n_drones+1):
            self.navigation.arm(id)
            print(f'[drone{id}] Armed.')
            self.navigation.takeoff(id, 5.0)

        time.sleep(40.0)  # time to go up

    def __init__(self):
        super().__init__('main')
        print("Node that reads the distances, computes the coordinates, plots the results and guides the drones.")

        # ROS parameters
        self.declare_parameter('n_drones', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('mode', rclpy.Parameter.Type.INTEGER)
        # self.declare_parameter('altitude', rclpy.Parameter.Type.DOUBLE)
        # self.declare_parameter('noise_dist_mean', rclpy.Parameter.Type.DOUBLE)
        # self.declare_parameter('noise_dist_std', rclpy.Parameter.Type.DOUBLE)
        # self.declare_parameter('noise_dyn_mean', rclpy.Parameter.Type.DOUBLE)
        # self.declare_parameter('noise_dyn_std', rclpy.Parameter.Type.DOUBLE)
        # self.declare_parameter('plot_type', rclpy.Parameter.Type.INTEGER)

        # Class attributes
        # received
        self.n_drones = self.get_parameter(
            'n_drones').get_parameter_value().integer_value
        self.mode = self.get_parameter(
            'mode').get_parameter_value().integer_value
        # self.altitude = self.get_parameter(
        #     'altitude').get_parameter_value().double_value
        # self.noise_dist_mean = self.get_parameter(
        #     'noise_dist_mean').get_parameter_value().double_value
        # self.noise_dist_std = self.get_parameter(
        #     'noise_dist_std').get_parameter_value().double_value
        # self.noise_dyn_mean = self.get_parameter(
        #     'noise_dyn_mean').get_parameter_value().double_value
        # self.noise_dyn_std = self.get_parameter(
        #     'noise_dyn_std').get_parameter_value().double_value
        # self.plot_type = self.get_parameter(
        #     'plot_type').get_parameter_value().string_value
        self.distances = np.ones((self.n_drones, self.n_drones))

        # calculated
        self.MDS_coords = np.ones(
            (3, self.n_drones))
        self.WLP_coords = np.ones(
            (3, self.n_drones))
        self.MDS_cov = np.ones((9, self.n_drones))

        # Topics
        # read the array of distances for each drone separatedly
        for i in range(self.n_drones):
            print(f"Topic registered to {DISTANCE_TOPIC_TEMPLATE(i+1)} to read ")
            self.create_subscription(
                Float32MultiArray,
                DISTANCE_TOPIC_TEMPLATE(i+1),
                lambda msg, i=i: self.distance_reader_callback(msg, i),
                qos_profile_system_default
            )

        # Initialize navigation
        self.navigation = Navigation()
        self.initialize_swarm()

        # Start cycle
        self.phase_index = 0    # for managing the execution
        self.anchor_id = 1
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
