import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class RangePublisher(Node):

    def __init__(self):
        super().__init__('range_publisher')
        self.publisher_ = self.create_publisher(String, 'tof8x8x3_msg', 10)
        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.range = -1

    def timer_callback(self):
        msg = String()
        range_string = 'TOF8x8x3'
        for i in range(192):
            range_string = range_string + ' ' + str(self.range + i*5)
        msg.data = range_string
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