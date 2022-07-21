import rclpy
from rclpy.node import Node
from .position import Position
from rcl_interfaces.msg import ParameterDescriptor

from std_msgs.msg import Int32MultiArray


class Rover(Node):

    def __init__(self):
        super().__init__('rover') #+ str(self.get_parameter("index")))

        param_descriptor1 = ParameterDescriptor(
            description = 'Position of the robot'
        )
        self.declare_parameter('position','{"x":2.0,"y":1.0,"z":0.0}',param_descriptor1)

        param_descriptor2 = ParameterDescriptor(
            description = 'ID of the robot'
        )
        self.declare_parameter('index',1,param_descriptor2)

        param_descriptor3 = ParameterDescriptor(
            description = 'Vector of squared euclidean distances of the current drone from all others'
        )
        self.declare_parameter('dists',[2,2],param_descriptor3)
        #self.get_logger().error(self.get_parameter('dists').get_parameter_value().double_array_value)

        self.index = self.get_parameter("index").value
        #self.position = self.get_parameter("position").get_parameter_value().double_array_value
        self.dists = self.get_parameter("dists").value
        self.publisher_ = self.create_publisher(Int32MultiArray, "/distances/" + self.get_name() , 10) 
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        self.get_logger().error(str(self.dists))
        msg = Int32MultiArray()
        msg.data = self.dists
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % ' ,'.join([str(elem) for elem in msg.data]))

    


def main(args=None):
    rclpy.init(args=args)

    rover = Rover()

    rclpy.spin(rover)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rover.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
