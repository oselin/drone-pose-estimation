#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Float32MultiArray

import time

from navigation import Navigation


def POSE_TOPIC_TEMPLATE(i): return f"/drone{i}/local_position/pose"
def DISTANCE_TOPIC_TEMPLATE(i): return f"/drone{i}/distances"


TIMESTEP = 0.5

MAP_COEFFICIENTS = [
    (1, 0, 0), 
    (0, 1, 0), 
    (0, 0, 1) 
]

VELOCITY_MAGNITUDE = 1.0 # [m/s]

class Main(Node):
    def distance_reader_callback(self, received_msg, index):
        print(received_msg.data)
        print(np.array(received_msg.data))
        self.distances[:, index] = np.array(received_msg.data)
        self.distances_booleans[index] = True
        self.get_logger().info('Received distances for drone%i: %s' %
                                (index+1, str(self.distances[:, index])))

    # def leggi e mds():
        # logica per gestire il movimento
        # calcolo distanza integrata 
        # Richiamo funzioni ALgebra.py

        # se passo moviment/direzione alora CHiamo MDS etc di algebra

        # se no nuovo algorimto con Exp-Min
    
    def guide_anchor(self, direction):
        if self.mode == 0:
            # move the anchor to a specific point and collect new distances from there
            # we assume this one is not possible given the list of commands provided by mavros
            # unless we consider an estimation of the position in this function
            # pos_x, pos_y, pos_z....
            # self.navigation.send_setpoint_position(self.anchor_id, coef_x)
            pass
        elif self.mode == 1:
            # move the anchor with a given velocity and collect new distances after a certain time
            vel_x, vel_y, vel_z = direction * VELOCITY_MAGNITUDE
            self.navigation.send_setpoint_velocity(self.anchor_id, vel_x, vel_y, vel_z, 0.0)
        elif self.mode == 2:
            # move the anchor in a certain direction and compute the estimation with what you obtain
            vel_x, vel_y, vel_z = direction * VELOCITY_MAGNITUDE
            self.navigation.send_setpoint_velocity(self.anchor_id, vel_x, vel_y, vel_z, 0.0)


    def cycle_callback(self):
        # guide all the drones

        # guide anchor
        direction = MAP_COEFFICIENTS[self.phase_id-1]
        self.guide_anchor(direction)
        # if self.guide_anchor(direction):
        #     # # moviment: se sappiamo quanto si è moss al'ancora
        #     # # tempo: se conmosciamo solo la direzione e dobbiamo integrare
        #     # # direzione: nel terzo caso
        #     # leggi e MDS(movimento)
        #     # update_plots()
        #     # index+=1 % 3    
        #     pass

    def check_distances(self):
        return np.all(self.distances_booleans)

    def __init__(self):
        super().__init__('main')
        self.get_logger().info(
            "Node that reads the distances, computes the coordinates, plots the results and guides the drones.")

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
        self.distances_booleans = np.tile(False, self.n_drones)

        # calculated
        self.MDS_coords = np.ones(
            (3, self.n_drones))
        self.WLP_coords = np.ones(
            (3, self.n_drones))
        self.MDS_cov = np.ones((9, self.n_drones))

        # Topics
        # read the array of distances for each drone separatedly
        for i in range(self.n_drones):
            print("Topic registered to %s to read " %
                  str(DISTANCE_TOPIC_TEMPLATE(i+1)))
            self.create_subscription(
                Float32MultiArray,
                DISTANCE_TOPIC_TEMPLATE(i+1),
                lambda msg, i=i: self.distance_reader_callback(msg, i),
                qos_profile_system_default
            )

        # Initialize navigation
        self.navigation = Navigation()
        for id in range(1, self.n_drones+1):
            self.navigation.set_mode(id, "GUIDED")
        
        time.sleep(5)

        for id in range(1, self.n_drones+1):
            self.navigation.arm(id)
        
        time.sleep(5)

        for id in range(1, self.n_drones+1):
            self.navigation.takeoff(id, 5.0) # self.altitude
        
        time.sleep(5)
        self.phase_id = 1    # for managing the execution
        self.anchor_id = 1

        # Start cycle
        self.get_logger().info("Waiting for distances..")
        while not self.check_distances():
            time.sleep(TIMESTEP)
        
        self.get_logger().info("Distances read")
        self.cycle_callback()
        
        # self.timer = self.create_timer(TIMESTEP, self.cycle_callback)


def main(args=None):
    rclpy.init(args=args)
    main = Main()
    rclpy.spin(main)

    main.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
