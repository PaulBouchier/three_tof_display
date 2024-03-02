import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16MultiArray


class RangePublisher(Node):

    def __init__(self):
        super().__init__('range_publisher')
        self.publisher_ = self.create_publisher(Int16MultiArray, 'range_array', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.range = -1

    def timer_callback(self):
        msg = Int16MultiArray()
        range_array = [self.range + i*10 for i in range(64)]
        msg.data = range_array
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing')
        self.range += 1000
        if self.range > 6000:
            self.range = -1


def main(args=None):
    rclpy.init(args=args)

    range_publisher = RangePublisher()

    rclpy.spin(range_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    range_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()