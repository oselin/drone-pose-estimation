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
from Plot import *


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
        service_string =  '/drone%i/cmd/arming' % id
        client = self.create_client(CommandBool, service_string)
        if not client.service_is_ready():
            self.get_logger().info('Service %s not available, waiting..' % service_string)
            client.wait_for_service(timeout_sec=5.0)

        request = CommandBool.Request()
        request.value = True

        repeating = True
        while repeating:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result is not None:
                if result.success:
                    repeating = False
                    self.get_logger().info('drone%i: Arming successful' % id)
                else:
                    time.sleep(5.0)
                    self.get_logger().info('drone%i: Arming failed' % id)
            else:
                repeating = False
                self.get_logger().info('drone%i: Failed to call Arming service' % id)

        client.destroy()

    def takeoff(self, id, altitude):
        service_string =  '/drone%i/cmd/takeoff' % id
        client = self.create_client(CommandTOL, service_string)
        while not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('drone%i: Service %s not available, waiting..' %(id, service_string))

        request = CommandTOL.Request()
        request.yaw = 0.0
        request.latitude = 0.0
        request.longitude = 0.0
        request.altitude = altitude
        request.min_pitch = 0.0
        
        repeating = True
        while repeating:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result is not None:
                if result.success:
                    repeating = False
                    self.get_logger().info(CGREEN2 + f"drone{id}: Takeoff successful" + CEND)
                else:
                    time.sleep(5.0)
                    self.get_logger().info(CRED2 + f"drone{id}: Takeoff failed" + CEND)
            else:
                repeating = False
                self.get_logger().info('Failed to call Takeoff service')

        client.destroy()

    def land(self, id):
        service_string =  '/drone%i/cmd/land' % id
        client = self.create_client(CommandTOL, service_string)
        while not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('drone%i: Service %s not available, waiting..' % (id, service_string))
        
        request = CommandTOL.Request()
        request.yaw = 0.0
        request.latitude = 0.0
        request.longitude = 0.0
        request.altitude = 0.0
        request.min_pitch = 0.0

        repeating = True
        while repeating:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result is not None:
                if result.mode_sent:
                    repeating = False
                    self.get_logger().info(CGREEN2 + f"drone{id}: Land successful" + CEND)
                else:
                    time.sleep(5.0)
                    self.get_logger().info(CRED2 + f"drone{id}: Land failed" + CEND)
            else:
                repeating = False
                self.get_logger().info('Failed to call Land service')
        client.destroy()

    def set_mode(self, id, mode):
        service_string = '/drone%i/set_mode' % id
        client = self.create_client(SetMode, service_string)
        while not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('drone%i: Service %s not available, waiting..' % (id, service_string))
        print(f'[drone{id}] Connection to {service_string} established.')
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = mode

        repeating = True
        while repeating:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result is not None:
                if result.mode_sent:
                    repeating = False
                    self.get_logger().info('drone%i: SetMode successful' % id)
                else:
                    time.sleep(5.0)
                    self.get_logger().info('drone%i: SetMode failed' % id)
            else:
                repeating = False
                self.get_logger().info('drone%i: Failed to call SetMode service' % id)
        
        client.destroy()

