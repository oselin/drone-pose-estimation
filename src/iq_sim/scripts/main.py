#!/usr/bin/env python3

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node
import rclpy

import numpy as np
import time
import threading


from Control import Navigation
import Algorithms
from Plot import class_name, Plot


def POSE_TOPIC_TEMPLATE(id): return f"/drone{id}/mavros/local_position/pose"
def DISTANCE_TOPIC_TEMPLATE(id): return f"/drone{id}/mavros/distances"


TIMESTEP = 0.1
CHECK_UPDATE_TIME = 5
ANCHOR_MOV_TIME = 1.0  # 1 s

SWARM_COEF = np.array([0.0, 1.0, 0.0])
ANCHOR_COEF = np.vstack([-np.eye(3), np.eye(3)])
# ANCHOR_COEF = np.array([
#     [0., 0., 0.],
#     [0., 0., 0.],
#     [1., 0., 0.],
#     [-1., 0., 0.],
#     [0., 1., 0.],
#     [0., -1., 0.],
#     [0., 0., 1.],
#     [0., 0., -1.],
# ])


SWARM_VEL = 0.2  # [m/s]
ANCHOR_VEL = 1.0


def M_ROT_TRASL_DRONE_GZ(i): return np.array(
    [[0, 1, 0, i], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def M_ROT_TRASL_DRONE_GZ(i): return np.eye(4)

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
        # self.get_logger().info(f"drone{index}: {str(self.coords[:, index])}")

    def distance_reader_callback(self, received_msg, index):
        """
        Update the Distance Matrix (DM) buffer by replacing the i-th columnn.
        """
        self.DM_buffer[:, index] = np.array(received_msg.data)

        if self.updating:
            self.update_booleans[index] = True

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

    def update(self):
        """
        Update the following class parameters:
            - anchor position
            - distance matrix
            - measurement index
            - phase index
            - movement time
            - timestamp
        """
        # New measure
        # practically
        prev_pos = self.PMs[:, self.meas_index - 1]
        anchor_mov = ANCHOR_COEF[self.phase_index] * \
            ANCHOR_VEL * self.anchor_timestep
        self.PMs[:, self.meas_index] = prev_pos + anchor_mov

        self.DMs[self.meas_index] = np.copy(self.DM_buffer)

        # ideally
        # # # self.DMs[self.meas_index] = Algorithms.distance_matrix(self.coords)
        # # # self.PMs[:, self.meas_index] = self.coords[:, 0]

        # Run algorithms
        X_mds, Cov_mds = self.MDS()
        X_wlp, Cov_wlp = self.WLP()

        # Update plots
        # if self.phase_index == 5:
        #     self.offset = self.PMs[:, self.meas_index]
        # else:
        #     self.offset += swarm_mov

        self.plot.update(
            true_coords=self.coords,
            MDS_coords=X_mds + self.offset.reshape(-1, 1),
            WLP_coords=X_wlp + self.offset.reshape(-1, 1),
            MDS_cov=None,
            WLP_cov=None
        )

        # Reset the booleans
        self.update_booleans[:] = False

        # Update cycle management
        self.meas_index = (self.meas_index + 1) % self.n_meas
        self.phase_index = (self.phase_index + 1) % len(ANCHOR_COEF)
        self.timestamp = self.get_timestamp()

    def check_update(self):
        """
        Check whether the distances vectors for all the drones have been received. In case, update the estimation and move the anchor.
        """
        if np.all(self.update_booleans):
            self.check_update_timer.cancel()
            self.update()
            self.updating = False

    def MDS(self):
        """
        Run MDS algorithm defined in the Algorithms class, by assembling the
        disance matrices in one unique [n+3, n+3] matrix.
        Return:
            - Coordinates of the drones swarm estimated via the algorithm.
        """
        # Shift based on the meas index
        DMs_tmp = np.roll(self.DMs, -self.meas_index, axis=0)
        PMs_tmp = np.roll(self.PMs, -self.meas_index, axis=1)

        # Assemble the full distance matrix
        DM = Algorithms.combine_matrices(
            DMs_tmp[0], DMs_tmp[1], DMs_tmp[2], DMs_tmp[3],
            PMs_tmp[:, 0], PMs_tmp[:, 1], PMs_tmp[:, 2], PMs_tmp[:, 3]
        )

        return Algorithms.MDS(DM, PMs_tmp), None

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

    def move_swarm(self, anchor):
        """
        Move the drone swarm by sending velcity values.
        The method does not affect the anchor motion.
        """
        # Compute the velocity components
        vel_x, vel_y, vel_z = SWARM_COEF*SWARM_VEL

        # Send velocity value to all the drones and to the anchor, in case it's flagged
        for id in range(2-int(anchor), self.n_drones+1):
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
        Node main loop. Update positions and perform measurments.
        """    
        if self.updating:
            # move all the drones simultaneously
            self.move_swarm(anchor=True)
        else:
            now_timestamp = self.get_timestamp()
            if ((now_timestamp - self.timestamp) >= self.mov_time):
                # block the possibility to perform another update and tell the cb to flag the booleans
                self.updating = True

                # make the swarm to keep going on
                self.move_swarm(anchor=True)

                # update the info to know how much the anchor has moved
                self.anchor_timestep = now_timestamp-self.timestamp

                # leave the time to update the distances and perform the update
                self.check_update_timer = self.create_timer(
                    CHECK_UPDATE_TIME, self.check_update
                )
            else:
                # move the swarm normally and the anchor to the new position
                self.move_swarm(anchor=False)
                self.move_anchor()

        # in any case the swarm has been moved
        self.offset += SWARM_COEF * SWARM_VEL * TIMESTEP

    def start(self):
        self.timer = self.create_timer(TIMESTEP, self.cycle_callback)
        self.a = self.get_timestamp()
        self.counter = 1
        self.timestamp = self.get_timestamp()
        self.start_timer.cancel()

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
        # management
        self.phase_index, self.meas_index = 0, 0
        self.anchor_id = 1
        self.n_meas = 4
        self.mov_time = ANCHOR_MOV_TIME
        self.algorithms = False
        self.updating = False

        self.update_booleans = np.zeros((self.n_drones,), dtype=bool)

        # measurements
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
            node=self, n_drones=self.n_drones, timeout=10
        )
        if (self.environment == "gazebo"):
            self.initialize_swarm()

        # Plotting
        self.plot = Plot(
            mode='2D',
            display_MDS=True,
            display_WLP=True,
            reduction_method='xy'
        )
        self.plot.start()

        # Time management
        def get_timestamp():
            now = self.get_clock().now().to_msg()
            return now.sec+now.nanosec/1e9
        self.get_timestamp = get_timestamp
        self.anchor_timestep = 0.0

        # Just wait some seconds (10) and start..
        self.start_timer = self.create_timer(10, self.start)


def main(args=None):
    rclpy.init(args=args)
    main = Main()
    rclpy.spin(main)

    main.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
