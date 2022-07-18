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

from std_msgs.msg import Float32MultiArray


class Anchor(Node):

    def __init__(self,position,number_of_drones):
        super().__init__('anchor')
        self.position = position
        self.number_of_drones = number_of_drones
        self.subscriptions = []
        self.d_mat = np.zeros((number_of_drones,number_of_drones))
        for i in range(self.number_of_drones):
            self.subscriptions.append(self.create_subscription(Float32MultiArray,
                                                                '/distances/rover' + str(i),
                                                                partial(self.listener_callback,i),
                                                                10))

    def listener_callback(self, msg, i):
        self.d_mat[i] = msg.data
        self.get_logger().info('I heard: "%s"' % ' '.join(msg.data))


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = Anchor()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
