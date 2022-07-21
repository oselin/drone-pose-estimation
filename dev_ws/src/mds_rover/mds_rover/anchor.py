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

from std_msgs.msg import Int32MultiArray



class Anchor(Node):

    def __init__(self):   #,
        super().__init__('anchor')

        param_descriptor1 = ParameterDescriptor(
            description = 'Position of the robot'
        )
        self.declare_parameter('position','{"x":0.0,"y":0.0,"z":0.0}',param_descriptor1)

        param_descriptor2 = ParameterDescriptor(
            description = 'Number of drones, not counting the anchor'
        )
        self.declare_parameter('number_of_drones','1',param_descriptor2)

        #print(self.get_parameters(["number_of_drones","position"]).__getattribute__())
        #self.position = self.get_parameter("position")
        self.number_of_drones = self.get_parameter("number_of_drones").get_parameter_value().integer_value
        print(str(self.number_of_drones))
        self.subs = []
        self.d_mat = np.empty((self.number_of_drones,self.number_of_drones), dtype=float)
        for i in range(self.number_of_drones):
            self.subs.append(self.create_subscription(Int32MultiArray,
                                                                '/distances/rover' + str(i),
                                                                partial(self.listener_callback,i),
                                                                10))

    def listener_callback(self, i, msg):
        print(msg.data)
        self.d_mat[i] = msg.data
        self.get_logger().info('I heard: "%s"' % ' '.join([str(elem)for elem in msg.data]))


def main(args=None):
    rclpy.init(args=args)

    anchor = Anchor()

    rclpy.spin(anchor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    anchor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
