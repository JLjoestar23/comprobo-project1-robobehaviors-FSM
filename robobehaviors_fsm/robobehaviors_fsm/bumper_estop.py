
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from neato2_interfaces.msg import Bump


class BumperEstop(Node):
    def __init__(self):
        super().__init__('bumper_estop')

        # publishers
        self.estop_pub = self.create_publisher(Bool, 'estop', 10)

        # subscriber
        self.create_subscription(Bump, '/bump', self.bumper_cb, 10)

        self.get_logger().info("BumperEstop node initialized")

    def bumper_cb(self, msg: Bump):
        # check bumper sensors
        bumped = bool(msg.left_front) or bool(msg.left_side) or bool(msg.right_front) or bool(msg.right_side)

        # publish estop signal only
        estop_msg = Bool()
        estop_msg.data = bumped
        self.estop_pub.publish(estop_msg)



def main(args=None):
    rclpy.init(args=args)
    node = BumperEstop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()