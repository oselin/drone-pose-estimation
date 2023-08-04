#!/usr/bin/env python3

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node
from Control.topics import *
from Control import Navigation
from Plot import class_name, Plot
import Algorithms
import rclpy
import time
import numpy as np


TIMESTEP = 0.1
CHECK_UPDATE_TIME = 5
ANCHOR_MOV_TIME = 1.0  # 1 s

SWARM_COEF = np.array([0.0, 1.0, 0.0])
ANCHOR_COEF = np.vstack([-np.eye(3), np.eye(3)])

SWARM_VEL = 0.2  # [m/s]
ANCHOR_VEL = 1.0


class Main(Node):

    def pose_reader_callback(self, received_msg, index):
        """
        Callback function for the POSE_TOPIC_TEMPLATE topic.
        Save the information sent over the topic in the coords data structure.
        It is activated only if 'environment' is set to 'test'
        """
        pos = received_msg.pose.position
        self.coords[:, index] = (Algorithms.M_ROT_TRASL_DRONE_GZ(
            index) @ np.array([pos.x, pos.y, pos.z, 1]))[:3]
        # self.get_logger().info(f"drone{index}: {str(self.coords[:, index])}")

    def distance_reader_callback(self, received_msg, index):
        """
        Update the Distance Matrix (DM) buffer by replacing the i-th columnn.
        """
        self.DM_buffer[:, index] = np.array(received_msg.data)

        if self.updating:
            self.update_booleans[index] = True

    def send_velocity(self, id, vel_x, vel_y, vel_z):
        # apply transformation to interact with Gazebo+Ardupilot
        M_GZ_DRONE = Algorithms.M_ROT_TRASL_GZ_DRONE(id-1)
        rel_vel = (M_GZ_DRONE @ np.array([vel_x, vel_y, vel_z, 0]))[:3]

        # send
        self.navigation.send_setpoint_velocity(
            id, rel_vel[0], rel_vel[1], rel_vel[2], 0.0
        )

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

        # set the z-coordinates of the anchor to "altitude", according to the takeoff
        self.PMs[2, 0] = self.altitude

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

        # Run the algorithm
        X_mds = Algorithms.MDS(DM, PMs_tmp)

        # Store the estimated coordinates
        self.X_mds_storage[self.mds_index] = X_mds

        # Compute the covaraince matrix
        if (not self.mds_index):
            Cov_mds = None
        else:
            Cov_mds = np.zeros((9, self.n_drones))
            for i in range(self.n_drones):
                Cov_mds[:, i] = np.cov(
                    self.X_mds_storage[:self.mds_index+1, :, i], rowvar=False).flatten()

        # Update the storage counter
        self.mds_index += 1

        return X_mds, Cov_mds

    def WLP(self):
        """
        Run WLP algorithm defined in the Algorithms class, by assembling the
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

        # Run the algorithm
        X_wlp = Algorithms.WLP(DM, PMs_tmp)

        # Store the estimated coordinates
        self.X_wlp_storage[self.wlp_index] = X_wlp

        # Compute the covaraince matrix
        if (not self.wlp_index):
            Cov_wlp = None
        else:
            Cov_wlp = np.zeros((9, self.n_drones))
            for i in range(self.n_drones):
                Cov_wlp[:, i] = np.cov(
                    self.X_wlp_storage[:self.wlp_index+1, :, i], rowvar=False).flatten()

        # Update the storage counter
        self.wlp_index += 1

        return X_wlp, Cov_wlp

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
        # The algorithms require at least 4 iterations.
        # Avoid executing the algorithms if less than 4 iter.
        if (not self.algorithms and self.phase_index > 3):
            self.algorithms = True

        # Update: x_(n) = x_(n-1) + Delta_x
        prev_pos = self.PMs[:, self.meas_index - 1]
        anchor_mov = ANCHOR_COEF[self.phase_index] * \
            ANCHOR_VEL * self.anchor_timestep

        self.PMs[:, self.meas_index] = prev_pos + anchor_mov

        # Store the just received distance matrix for the i-th (meas_index) iteration
        self.DMs[self.meas_index] = np.copy(self.DM_buffer)

        if (self.algorithms):

            # Run algorithms
            X_mds, Cov_mds = self.MDS()
            X_wlp, Cov_wlp = self.WLP()

            # Update the plot
            self.plot.update(
                true_coords=self.coords,
                MDS_coords=X_mds + self.offset.reshape(-1, 1),
                WLP_coords=X_wlp + self.offset.reshape(-1, 1),
                MDS_cov=Cov_mds,
                WLP_cov=Cov_wlp
            )

        # Reset the booleans
        self.update_booleans[:] = False

        # Update cycle management
        self.meas_index = (self.meas_index + 1) % self.n_meas
        self.phase_index = (self.phase_index + 1) % len(ANCHOR_COEF)
        self.anchor_timestamp = self.get_timestamp()

    def check_update(self):
        """
        Check whether the distances vectors for all the drones have been received. In case, update the estimation and move the anchor.
        """
        if np.all(self.update_booleans):
            self.check_update_timer.cancel()
            self.update()
            self.updating = False

    def move_swarm(self, anchor):
        """
        Move the drone swarm by sending velcity values.
        The method affects the anchor motion if anchor set to True.
        """
        # Compute the velocity components
        vel_x, vel_y, vel_z = SWARM_COEF*SWARM_VEL

        # Send velocity value to all the drones and to the anchor, in case it's flagged
        for id in range(2-int(anchor), self.n_drones+1):
            self.send_velocity(id, vel_x, vel_y, vel_z)

    def move_anchor(self):
        """
        Move the anchor dron by sending velcity values.
        The method does not affect the swarm motion.
        """
        # Compute the velocity components
        vel_x, vel_y, vel_z = SWARM_COEF * SWARM_VEL + \
            ANCHOR_COEF[self.phase_index] * ANCHOR_VEL

        # Send velocity value
        self.send_velocity(self.anchor_id, vel_x, vel_y, vel_z)

    def cycle_callback(self):
        """
        Node main loop. Update positions and perform measurments.
        """
        now_timestamp = self.get_timestamp()
        if self.updating:
            # move all the drones simultaneously
            self.move_swarm(anchor=True)
        else:
            if ((now_timestamp - self.anchor_timestamp) >= self.mov_time):
                # block the possibility to perform another update and tell the cb to flag the booleans
                self.updating = True

                # make the swarm to keep going on
                self.move_swarm(anchor=True)

                # update the info to know how much the anchor has moved
                self.anchor_timestep = now_timestamp-self.anchor_timestamp

                # leave the time to update the distances and perform the update
                self.check_update_timer = self.create_timer(
                    CHECK_UPDATE_TIME, self.check_update
                )
            else:
                # move the swarm normally and the anchor to the new position
                self.move_swarm(anchor=False)
                self.move_anchor()

        # take into account the real time elapsed from the previous movement
        timestep = now_timestamp - self.timestamp
        self.timestamp = now_timestamp

        # update swarm position by integration
        self.offset += SWARM_COEF * SWARM_VEL * timestep

    def start(self):
        self.timestamp = self.get_timestamp()
        self.anchor_timestamp = self.timestamp

        self.move_swarm(anchor=True)

        self.timer = self.create_timer(TIMESTEP, self.cycle_callback)
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
        self.mds_index = 0
        self.wlp_index = 0

        # measurements
        self.coords = np.zeros((3, self.n_drones))
        self.offset = np.zeros((3,))
        self.PMs = np.zeros((3, self.n_meas))
        self.DMs = np.zeros((self.n_meas, self.n_drones, self.n_drones))
        self.DM_buffer = np.zeros((self.n_drones, self.n_drones))
        self.X_mds_storage = np.zeros((1000, 3, self.n_drones))
        self.X_wlp_storage = np.zeros((1000, 3, self.n_drones))

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
        self.navigation = Navigation(node=self, n_drones=self.n_drones, timeout=10
                                     )
        if (self.environment == "gazebo"):
            self.initialize_swarm()

        # Plotting
        self.plot = Plot(
            mode='2D',
            display_MDS=True,
            display_WLP=True,
            reduction_method='xy',
            display_covariance=True,
        )
        self.plot.start()

        # Time management
        def get_timestamp():
            now = self.get_clock().now().to_msg()
            return now.sec+now.nanosec/1e9

        self.get_timestamp = get_timestamp
        self.timestamp = 0.0        # for each cycle
        self.anchor_timestep = 0.0
        self.anchor_timestamp = 0.0  # for the anchor movement

        # Just wait some seconds (10) and start..
        # Needed to allow a better estimation of the anchor movement
        self.start_timer = self.create_timer(10, self.start)


def main(args=None):
    rclpy.init(args=args)
    main = Main()
    rclpy.spin(main)

    main.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
