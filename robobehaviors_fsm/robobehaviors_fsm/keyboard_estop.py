import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import sys
import termios
import tty


class KeyboardEstop(Node):
    def __init__(self):
        super().__init__('keyboard_estop')

        # publisher for estop
        self.estop_pub = self.create_publisher(Bool, 'estop', 10)

        # track estop state
        self.estop_active = False

        self.get_logger().info("press space to toggle estop, q to quit")

        # timer to keep publishing estop state
        self.create_timer(0.1, self.publish_estop)

    def publish_estop(self):
        """publish the current estop state"""
        msg = Bool()
        msg.data = self.estop_active
        self.estop_pub.publish(msg)

    def key_loop(self):
        """listen for keyboard input to toggle estop"""
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                key = sys.stdin.read(1)
                if key == ' ':
                    self.estop_active = not self.estop_active
                    state = "on" if self.estop_active else "off"
                    self.get_logger().warn(f"keyboard estop toggled {state}")
                elif key == 'q':
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardEstop()
    try:
        node.key_loop()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
