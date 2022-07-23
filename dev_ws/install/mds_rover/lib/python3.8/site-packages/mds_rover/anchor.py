# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import numpy as np
from rclpy.node import Node
from functools import partial
from rcl_interfaces.msg import ParameterDescriptor

from std_msgs.msg import Float32MultiArray

from .position import Position


class Anchor(Node):

    def __init__(self):  # ,
        super().__init__('anchor')

        self.declare_parameter(
            'position',
            None,
            ParameterDescriptor(description='Position of the robot')
        )

        self.declare_parameter(
            'number_of_nodes',
            None,
            ParameterDescriptor(
                description='Number of robots')
        )

        self.position = self.get_parameter("position").value
        self.number_of_nodes = self.get_parameter("number_of_nodes").value

        # compute the distances, as for the other robots
        self.dist = [0.0,0.7,0.7,0.7,0.7]

        # listen to the topics for the distances
        self.received = np.empty(self.number_of_nodes, dtype=bool)
        self.d_mat = np.empty(
            (self.number_of_nodes, self.number_of_nodes), 
            dtype=float
        )
        self.d_mat[0] = self.dist
        for i in range(1, self.number_of_nodes):
            self.create_subscription(
                Float32MultiArray,
                '/rover' + str(i) + '/distances',
                partial(self.listener_callback, i),
                10
            )
        

    def listener_callback(self, i, msg):
        self.get_logger().info(
            'Received message from '+str(i)+': ' +
            ' '.join(str(elem) for elem in msg.data))
        self.d_mat[i] = msg.data
        self.received[i] = True
        self.mds()

    def mds(self):
        print("chec")
        # chech if the matrix contains a row for each drone
        if not all(fb == True for fb in self.received):
            return
        self.get_logger().info("MDS calculated on a full matrix")


def main(args=None):
    rclpy.init(args=args)
    anchor = Anchor()
    rclpy.spin(anchor)
    anchor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
