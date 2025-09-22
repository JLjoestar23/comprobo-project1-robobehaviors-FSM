import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class BumperEstop(Node):
    def __init__(self):
        super().__init__('bumper_estop')

        # publisher for estop
        self.estop_pub = self.create_publisher(Bool, 'estop', 10)

        # subscribe to bumper sensor (assumed to be publishing bool on /bumper_hit)
        self.create_subscription(Bool, 'bumper_hit', self.bumper_cb, 10)

    def bumper_cb(self, msg: Bool):
        """forward bumper state to estop"""
        self.estop_pub.publish(msg)

        if msg.data:
            self.get_logger().warn("bumper estop activated")
        else:
            self.get_logger().info("bumper estop cleared")


def main(args=None):
    rclpy.init(args=args)
    node = BumperEstop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
