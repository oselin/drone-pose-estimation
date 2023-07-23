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

class DroneController(Node):
    def __init__(self,drone):
        super().__init__('drone_controller')
        self.setpoint_pub = self.create_publisher(PoseStamped, '/' + drone + '/setpoint_position/local', 10)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/' + drone + '/setpoint_velocity/cmd_vel', 10)
        self.arm_client = self.create_client(CommandBool, '/' + drone + '/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/' + drone + '/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/' + drone + '/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/' + drone + '/cmd/land')
        self.mode_client = self.create_client(SetMode, '/' + drone + '/set_mode')

    def send_setpoint_position(self, x, y, z):
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        self.setpoint_pub.publish(pose_msg)

    def send_setpoint_velocity(self, linear_x, linear_y, linear_z, angular_z):
        twist_msg = TwistStamped()
        twist_msg.twist.linear.x = linear_x
        twist_msg.twist.linear.y = linear_y
        twist_msg.twist.linear.z = linear_z
        twist_msg.twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist_msg)

    def arm(self):
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Arming service not available, waiting...')
        request = CommandBool.Request()
        request.value = True
        future = self.arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Arming successful')
            else:
                self.get_logger().info('Arming failed')
        else:
            self.get_logger().info('Failed to call arming service')
            
    def takeoff(self, altitude):
        #while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('Takeoff service not available, waiting...')
        takeoff_srv = CommandTOL.Request()
        takeoff_srv.yaw = 0.0
        takeoff_srv.latitude = 0.0
        takeoff_srv.longitude = 0.0
        takeoff_srv.altitude = altitude
        takeoff_srv.min_pitch = 0.0

        future = self.takeoff_client.call_async(takeoff_srv)
        rclpy.spin_until_future_complete(self, future)        
        if future.result():
            self.get_logger().info(CGREEN2 + "Takeoff successful" + CEND)
            return 0
        else:
            self.get_logger().info(CRED2 + "Takeoff failed" + CEND)
            return -1
        
    def land(self):
        #while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('Takeoff service not available, waiting...')
        takeoff_srv = CommandTOL.Request()
        takeoff_srv.yaw = 0.0
        takeoff_srv.latitude = 0.0
        takeoff_srv.longitude = 0.0
        takeoff_srv.altitude = 0.0
        takeoff_srv.min_pitch = 0.0
        future = self.land_client.call(takeoff_srv)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.get_logger().info(CGREEN2 + "Land successful" + CEND)
            return 0
        else:
            self.get_logger().info(CRED2 + "Land failed" + CEND)
            return -1

    def set_mode(self, mode):
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetMode service not available, waiting...')
        request = SetMode.Request()
        request.custom_mode = mode
        future = self.mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().mode_sent:
                self.get_logger().info('SetMode successful')
            else:
                self.get_logger().info('SetMode failed')
        else:
            self.get_logger().info('Failed to call SetMode service')

def main(args=None):
    rclpy.init(args=args)
    
    
    drone_1_ctrl = DroneController("drone1")
    #drone_2_ctrl = DroneController("drone2")

    
    drone_1_ctrl.set_mode('GUIDED')
    #drone_2_ctrl.set_mode('GUIDED')
    
    time.sleep(5)
    
    # Arming and setting the mode to OFFBOARD
    drone_1_ctrl.arm()
    #drone_2_ctrl.arm() 
    
    time.sleep(3)
    
    drone_1_ctrl.takeoff(5.0)
    #drone_2_ctrl.takeoff(5.0) 

    # Move the drone by sending commands
    drone_1_ctrl.send_setpoint_position(1.0, 0.0, 2.0)  
    #drone_2_ctrl.send_setpoint_position(3.0, 0.0, 2.0)  
    
    time.sleep(10)
    
    #drone_controller.send_setpoint_velocity(0.0, 0.0, 1.0, 0.0)  # Move with linear z velocity of 1 m/s

    drone_1_ctrl.land()
    #drone_2_ctrl.land()
    
    time.sleep(5)
    
    rclpy.spin(drone_1_ctrl)
    #rclpy.spin(drone_2_ctrl)

    drone_1_ctrl.destroy_node()
    #drone_2_ctrl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()













































# #! /usr/bin/env python
# # Import ROS.
# import rospy
# # Import the API.
# from iq_gnc.py_gnc_functions import *
# # To print colours (optional).
# from iq_gnc.PrintColours import *


# def main():
#     # Initializing ROS node.
#     rospy.init_node("drone_controller", anonymous=True)

#     # Create an object for the API.
#     drone = gnc_api()
#     # Wait for FCU connection.
#     drone.wait4connect()
#     # Wait for the mode to be switched.
#     drone.wait4start()

#     # Create local reference frame.
#     drone.initialize_local_frame()
#     # Request takeoff with an altitude of 3m.
#     drone.takeoff(3)
#     # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
#     rate = rospy.Rate(3)

#     # Specify some waypoints
#     goals = [[0, 0, 3, 0], [5, 0, 3, -90], [5, 5, 3, 0],
#              [0, 5, 3, 90], [0, 0, 3, 180], [0, 0, 3, 0]]
#     i = 0

#     while i < len(goals):
#         drone.set_destination(
#             x=goals[i][0], y=goals[i][1], z=goals[i][2], psi=goals[i][3])
#         rate.sleep()
#         if drone.check_waypoint_reached():
#             i += 1
#     # Land after all waypoints is reached.
#     drone.land()
#     rospy.loginfo(CGREEN2 + "All waypoints reached landing now." + CEND)


# if __name__ == '__main__':
#     try:
#         main()
#     except KeyboardInterrupt:
#         exit()
