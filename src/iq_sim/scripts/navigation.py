#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import SetMode, CommandBool
import time
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import CommandLong
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode


# Import the API.
from PrintColours import *


class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')

    def send_setpoint_position(self, id, x, y, z):
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        self.create_publisher(
            PoseStamped, '/drone%i/setpoint_position/local' % id, 10).publish(pose_msg)

    def send_setpoint_velocity(self, id, linear_x, linear_y, linear_z, angular_z):
        twist_msg = TwistStamped()
        twist_msg.twist.linear.x = linear_x
        twist_msg.twist.linear.y = linear_y
        twist_msg.twist.linear.z = linear_z
        twist_msg.twist.angular.z = angular_z
        self.create_publisher(
            TwistStamped, '/drone%i/setpoint_velocity/cmd_vel' % id, 10).publish(twist_msg)

    def arm(self, id):
        arm_client = self.create_client(
            CommandBool, '/drone%i/cmd/arming' % id)

        while not arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Arming service not available, waiting...')
        request = CommandBool.Request()
        request.value = True
        future = arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Arming successful')
            else:
                self.get_logger().info('Arming failed')
        else:
            self.get_logger().info('Failed to call arming service')

    def takeoff(self, id, altitude):
        # while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('Takeoff service not available, waiting...')
        takeoff_srv = CommandTOL.Request()
        takeoff_srv.yaw = 0.0
        takeoff_srv.latitude = 0.0
        takeoff_srv.longitude = 0.0
        takeoff_srv.altitude = altitude
        takeoff_srv.min_pitch = 0.0

        future = self.create_client(
            CommandTOL, '/drone%i/cmd/takeoff' % id).call_async(takeoff_srv)
        # rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(CGREEN2 + "Takeoff successful" + CEND)
            return 0
        else:
            self.get_logger().info(CRED2 + "Takeoff failed" + CEND)
            return -1

    def land(self, id):
        # while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('Takeoff service not available, waiting...')
        takeoff_srv = CommandTOL.Request()
        takeoff_srv.yaw = 0.0
        takeoff_srv.latitude = 0.0
        takeoff_srv.longitude = 0.0
        takeoff_srv.altitude = 0.0
        takeoff_srv.min_pitch = 0.0
        future = self.create_client(
            CommandTOL, '/drone%i/cmd/land' % id).call_async(takeoff_srv)
        # rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(CGREEN2 + "Land successful" + CEND)
            return 0
        else:
            self.get_logger().info(CRED2 + "Land failed" + CEND)
            return -1

    def set_mode(self, id, mode):
        mode_client = self.create_client(
            SetMode, '/drone%i/set_mode' % id)
        while not mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetMode service not available, waiting...')
        request = SetMode.Request()
        request.custom_mode = mode
        future = mode_client.call_async(request)
        # rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().mode_sent:
                self.get_logger().info('SetMode successful')
            else:
                self.get_logger().info('SetMode failed')
        else:
            self.get_logger().info('Failed to call SetMode service')


    # move_pos()
    #     # necesitta di conoscere pos

    # move_vel()  
    #     send_setpoint_velocity(+tempo)