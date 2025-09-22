import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class FiniteStateController(Node):
    def __init__(self):
        super().__init__('finite_state_controller')

        # publisher for current state
        self.state_pub = self.create_publisher(String, 'current_state', 10)

        # subscribers for triggers
        self.create_subscription(Bool, 'wall_detected', self.wall_sub_cb, 10)
        self.create_subscription(Bool, 'person_detected', self.person_sub_cb, 10)
        self.create_subscription(Bool, 'estop', self.estop_sub_cb, 10)

        # initial state
        self.current_state = "wall_following"
        self.publish_state()
        
        self.timer = self.create_timer(1.0, self.loop_cb)


    def loop_cb(self):
        """periodic loop to print state subs"""
        self.get_logger().info(f"current state: {self.current_state}")
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)
        self.get_logger().info(f"Publishing, current state: {self.current_state}")

        
        
        
    def publish_state(self):
        """publish the current fsm state"""
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)
        self.get_logger().info(f"Publishing, current state: {self.current_state}")

    def wall_sub_cb(self, msg: Bool):
        """handle wall detection events"""
        if self.current_state == "estop":
            return

        wall_detected = msg.data

        if wall_detected and self.current_state == "obstacle_avoidance":
            self.current_state = "wall_following"
            self.publish_state()
            self.get_logger().info("transitioned to wall following state")

        elif not wall_detected and self.current_state == "wall_following":
            self.current_state = "obstacle_avoidance"
            self.publish_state()
            self.get_logger().info("wall lost, back to obstacle avoidance state")

    def person_sub_cb(self, msg: Bool):
        """handle person detection events"""
        if self.current_state == "estop":
            return

        person_detected = msg.data

        if person_detected and self.current_state == "wall_following":
            self.current_state = "person_following"
            self.publish_state()
            self.get_logger().info("transitioned from wall following to person following")

        elif not person_detected and self.current_state == "person_following":
            self.current_state = "wall_following"
            self.publish_state()
            self.get_logger().info("person lost, back to wall following state")

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


def main(args=None):
    rclpy.init(args=args)
    node = FiniteStateController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
