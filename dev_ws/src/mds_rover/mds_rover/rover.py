import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Float32MultiArray

from .position import Position


class Rover(Node):

    def __init__(self):
        super().__init__('rover')  # + str(self.get_parameter("index")))

        self.declare_parameter(
            'position',
            None,
            ParameterDescriptor(description='Position of the robot')
        )

        self.declare_parameter(
            'index',
            None,
            ParameterDescriptor(description='Index of the robot')
        )

        self.declare_parameter(
            'dists',
            None,
            ParameterDescriptor(
                description='Vector of distances from all others robots')
        )

        self.index = self.get_parameter("index").value
        self.position = self.get_parameter("position").value
        self.dists = self.get_parameter("dists").value

        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            self.get_namespace()+"/distances",
            10
        )
        timer_period = 5  # seconds
        #for i in range(5):
        #self.publish()
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
    #def publish(self):
        msg = Float32MultiArray()
        msg.data = self.dists
        self.publisher_.publish(msg)
        self.get_logger().info(
            'Publishing: "%s"' % ' '.join([str(elem) for elem in msg.data])
        )


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
