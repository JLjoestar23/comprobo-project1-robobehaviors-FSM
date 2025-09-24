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
        self.create_subscription(Bool, "/wall_detected", self.wall_sub_cb, 10)
        self.create_subscription(Bool, "/estop", self.estop_sub_cb, 10)
        self.create_subscription(Bool, "/teleop_toggle", self.teleop_sub_cb, 10)  # listen for teleop key
        self.create_subscription(Bool, "/target_reached", self.obstacle_avoidance_cb, 10)

        # conditions for each state
        self.wall_detected = None
        self.estop = None
        self.teleop_toggle = None
        self.target_reached = None

        # initial state
        # "obstacle_avoidance", "wall_following", "teleop", "estop"
        self.current_state = "obstacle_avoidance"
        self.publish_state()
        
        self.timer = self.create_timer(0.1, self.evaluate_state)

    def evaluate_state(self):
        """periodic loop to publish state"""
        
        if self.estop:
            self.current_state = "estop"

        match self.current_state:
            case "obstacle_avoidance":
                if self.target_reached and self.wall_detected:
                    self.current_state = "wall_following"
                    print(self.current_state)
            case "wall_following":
                if self.teleop_toggle:
                    self.current_state = "teleop"
                    self.publish_zero_velocity()
                    print(self.current_state)
            case "teleop":
                if not self.teleop_toggle and not self.estop:
                    self.current_state = "wall_following"
                    self.publish_zero_velocity()
                    print(self.current_state)
            case "estop":
                self.publish_zero_velocity()
                if not self.estop and self.teleop_toggle:
                    self.current_state = "teleop"
                    print(self.current_state)

        self.publish_state()

    def publish_state(self):
        """publish the current fsm state"""
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)
        #self.get_logger().info(f"Publishing, current state: {self.current_state}")

    def publish_zero_velocity(self):
        """publish a Twist with all zeros to stop the robot"""
        stop_msg = Twist()  # defaults to all zeros
        self.cmd_pub.publish(stop_msg)
        self.get_logger().warn("!!! publishing zero velocity command !!!")

    def obstacle_avoidance_cb(self, msg: Bool):
        self.target_reached = msg.data

    def wall_sub_cb(self, msg: Bool):
        """handle wall detection events"""
        self.wall_detected = msg.data

    def estop_sub_cb(self, msg: Bool):
        """handle estop events from bumper or keyboard"""
        self.estop = msg.data

    def teleop_sub_cb(self, msg: Bool):
        """handle teleop mode request from keypress"""
        self.teleop_toggle = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = FiniteStateController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
