#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import SetMode, CommandBool
import time
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from rclpy.qos import qos_profile_system_default

# Import the API.
from Plot.PrintColours import *


def POSITION_TOPIC_TEMPLATE(id): return f'/drone{id}/mavros/setpoint_position/local'
def VELOCITY_TOPIC_TEMPLATE(id): return f'/drone{id}/mavros/setpoint_velocity/cmd_vel'
def SETMODE_TOPIC_TEMPLATE(id):  return f'/drone{id}/mavros/set_mode'
def ARMING_TOPIC_TEMPLATE(id):   return f'/drone{id}/mavros/cmd/arming'
def TAKEOFF_TOPIC_TEMPLATE(id):  return f'/drone{id}/mavros/cmd/takeoff'
def LAND_TOPIC_TEMPLATE(id):     return f'/drone{id}/mavros/cmd/land'


class Navigation(Node):

    def __init__(self, n_drones, timeout=10):
        super().__init__('navigation')
        self.timeout = timeout

        self.pos_publisher, self.vel_publisher = [], []

        for id in range(1,n_drones+1):
            self.pos_publisher.append(
                self.create_publisher(PoseStamped, POSITION_TOPIC_TEMPLATE(id),qos_profile_system_default))
            self.vel_publisher.append(
                self.create_publisher(TwistStamped, VELOCITY_TOPIC_TEMPLATE(id), qos_profile_system_default))


    def __call_service(self, client, request, service_name, drone_id):
        """
        Service caller: wait for services to be available and forward the desired request.
        Parameters:
            - client: agent that allows the communication
            - request: content of the request that has to be sent
            - service_name: name of service for logging purposes
            - drone_id: drone id for logging purposes
        """
        while not client.wait_for_service(timeout_sec=self.timeout):
            self.get_logger().info(f'drone{drone_id}: {service_name} service not available, waiting..')

        while True:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if (result is not None):
                if (result.success):
                    self.get_logger().info(CGREEN2 + f"drone{drone_id}: {service_name} successful" + CEND)
                    break
                else:
                    self.get_logger().info(CRED2 + f"drone{drone_id}: {service_name} failed" + CEND)
                    time.sleep(self.timeout)
            else:
                self.get_logger().info(f'Failed to call {service_name} service')
                break

            client.remove_pending_request(future)
        client.destroy()
    
    
    def send_setpoint_position(self, id, x, y, z):
        """
        Send position coordinates to POSITION_TOPIC_TEMPLATE topic.
        Parameters:
            - id: targeted drone to be commanded
            - x: X-cartesian coordinate
            - y: Y-cartesian coordinate
            - z: Z-cartesian coordinate
        """
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z

        # Send the message
        self.pos_publisher[id-1].publish(pose_msg)

    
    def send_setpoint_velocity(self, id, linear_x, linear_y, linear_z, angular_z):
        """
        Send position coordinates to VELOCITY_TOPIC_TEMPLATE topic.
        """
        twist_msg = TwistStamped()
        twist_msg.twist.linear.x = linear_x
        twist_msg.twist.linear.y = linear_y
        twist_msg.twist.linear.z = linear_z
        twist_msg.twist.angular.z = angular_z

        # Send the message
        self.vel_publisher[id-1].publish(twist_msg)
    

    def set_mode(self, id, mode):
        """
        Let the drone id change mode and take the desired one.
        Parameters:
            - id: targeted drone to be commanded
            - mode: the desired mode to be applied
        """
        # Create client
        client = self.create_client(SetMode, SETMODE_TOPIC_TEMPLATE(id))

        # Create request
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = mode
        
        # Call the service
        self.__call_service(client=client, request=request, service_name='SETMODE', drone_id=id)


    def arm(self, id):
        """
        Let the drone id arm the throttles
        Parameters:
            - id: targeted drone to be commanded
        """
        # Create client
        client = self.create_client(CommandBool, ARMING_TOPIC_TEMPLATE(id))

        # Create Request
        request = CommandBool.Request()
        request.value = True

        # Call the service
        self.__call_service(client=client, request=request, service_name='ARM', drone_id=id)


    def takeoff(self, id, altitude):
        """
        Let the drone id take off and reach the desired altitude.
        Parameters:
            - id: targeted drone to be commanded
            - altitude: desired altitude to be reached
        """
        # Create client
        client = self.create_client(CommandTOL, TAKEOFF_TOPIC_TEMPLATE(id))

        # Create request
        request = CommandTOL.Request()
        request.altitude = altitude

        # Call the service
        self.__call_service(client=client, request=request, service_name='TAKEOFF', drone_id=id)


    def land(self, id):
        """
        Let the drone id land.
        Parameters:
            - id: targeted drone to be commanded
        """
        # Create client
        client = self.create_client(CommandTOL, LAND_TOPIC_TEMPLATE(id))
        
        # Create request - the default one works
        request = CommandTOL.Request()

        # Call the service
        self.__call_service(client=client, request=request, service_name='LAND', drone_id=id)
        

