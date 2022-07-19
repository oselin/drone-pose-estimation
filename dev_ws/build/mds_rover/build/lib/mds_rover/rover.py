import rclpy
from rclpy.node import Node
from .position import Position
from rcl_interfaces.msg import ParameterDescriptor

from std_msgs.msg import Float32MultiArray


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
        self.declare_parameter('index','1',param_descriptor2)

        print(self.get_parameters(["index","position"]))
        self.index = self.get_parameter("index")
        print(self.index)
        self.publisher_ = self.create_publisher(Float32MultiArray, "/distances/" + self.get_name() , 10) #+ str(self.index)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = [1.0,1.0,1.0]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    


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
