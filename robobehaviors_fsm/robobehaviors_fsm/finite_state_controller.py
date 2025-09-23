import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist   # NEW import


class FiniteStateController(Node):
    def __init__(self):
        super().__init__('finite_state_controller')

        # publisher for current state
        self.state_pub = self.create_publisher(String, 'current_state', 10)

        # publisher for velocity commands (stop robot on estop)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)  # NEW

        # subscribers for triggers
        self.create_subscription(Bool, 'wall_detected', self.wall_sub_cb, 10)
        self.create_subscription(Bool, 'estop', self.estop_sub_cb, 10)
        self.create_subscription(Bool, 'teleop_toggle', self.teleop_sub_cb, 10)  # listen for teleop key

        # initial state
        self.current_state = "wall_following"
        self.publish_state()
        
        self.timer = self.create_timer(1.0, self.loop_cb)

    def loop_cb(self):
        """periodic loop to publish state"""
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)

        # continuously enforce stop if in estop
        if self.current_state == "estop":
            self.publish_zero_velocity()

    def publish_state(self):
        """publish the current fsm state"""
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)
        self.get_logger().info(f"Publishing, current state: {self.current_state}")

        # immediately stop robot if estop
        if self.current_state == "estop":
            self.publish_zero_velocity()

    def publish_zero_velocity(self):
        """publish a Twist with all zeros to stop the robot"""
        stop_msg = Twist()  # defaults to all zeros
        self.cmd_pub.publish(stop_msg)
        self.get_logger().warn("!!! publishing zero velocity command !!!")

    def wall_sub_cb(self, msg: Bool):
        """handle wall detection events"""
        if self.current_state == "estop":
            return

        wall_detected = msg.data

        if wall_detected:
            self.current_state = "wall_following"
            self.publish_state()
            self.get_logger().info("transitioned to wall following state")

        elif not wall_detected and self.current_state == "wall_following":
            self.publish_state()
            self.get_logger().info("wall lost, back to obstacle avoidance state")

    def estop_sub_cb(self, msg: Bool):
        """handle estop events from bumper or keyboard"""
        estop_triggered = msg.data

        if estop_triggered and self.current_state != "estop":
            self.current_state = "estop"
            self.publish_state()
            self.get_logger().warn("!!! emergency stop activated !!!")

        elif not estop_triggered and self.current_state == "estop":
            self.current_state = "obstacle_avoidance"
            self.publish_state()
            self.get_logger().info("estop cleared, back to obstacle avoidance state")

    def teleop_sub_cb(self, msg: Bool):
        """handle teleop mode request from keypress"""
        teleop_requested = msg.data

        if teleop_requested and self.current_state != "estop":
            self.current_state = "teleop"
            self.publish_state()
            self.get_logger().info("transitioned to teleop state")


def main(args=None):
    rclpy.init(args=args)
    node = FiniteStateController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
