import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from threading import Lock


class FiniteStateController(Node):
    """
    Main FSM node that coordinates robot behaviors and handles state transitions.
    
    Acts as central command arbiter - only this node publishes to /cmd_vel
    to prevent conflicting commands from multiple behaviors.
    """
    
    def __init__(self):
        super().__init__('finite_state_controller')
        
        # Current state
        self.current_state = "obstacle_avoidance"
        self.previous_state = ""
        
        # Publishers -- cmd_vel is the ONLY publisher to velocity in the entire workspace
        self.state_publisher = self.create_publisher(String, 'current_state', 10)
        self.robot_cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)  
        
        # velocity command subscribers from each node
        self.create_subscription(Twist, "/obstacle_avoidance_cmd", self.handle_obstacle_cmd, 10)
        self.create_subscription(Twist, "/wall_following_cmd", self.handle_wall_cmd, 10)
        self.create_subscription(Twist, "/teleop_cmd", self.handle_teleop_cmd, 10)

        # State trigger function subs
        self.create_subscription(Bool, "/wall_detected", self.on_wall_detected, 10)
        self.create_subscription(Bool, "/estop", self.on_estop_triggered, 10)
        self.create_subscription(Bool, "/teleop_toggle", self.on_teleop_pressed, 10)
        self.create_subscription(Bool, "/target_reached", self.on_waypoint_reached, 10)

        # thread lock to prevent touching variables at same time
        self.state_lock = Lock()

        self.wall_is_detected = False
        self.emergency_stop_active = False
        self.teleop_key_pressed = False
        self.waypoint_reached = False
        
        self.last_teleop_state = False
        self.latest_obstacle_cmd = Twist()
        self.latest_wall_cmd = Twist()  
        self.latest_teleop_cmd = Twist()

        # Timers
        self.state_update_timer = self.create_timer(0.05, self.update_state_machine)  
        self.command_publisher_timer = self.create_timer(0.1, self.publish_active_command)  
        
        # Initialize
        self.broadcast_current_state()
        self.get_logger().info(f"FSM started in state: {self.current_state}")

    def update_state_machine(self):
        with self.state_lock:
            teleop_just_pressed = self.teleop_key_pressed and not self.last_teleop_state

            if self.emergency_stop_active and self.current_state != "estop":
                self.transition_to("estop")
                self.last_teleop_state = self.teleop_key_pressed
                self.broadcast_current_state()
                return

            match self.current_state:
                case "obstacle_avoidance":
                    if self.waypoint_reached and self.wall_is_detected:
                        self.transition_to("wall_following")
                case "wall_following":
                    if teleop_just_pressed:
                        self.transition_to("teleop")
                case "teleop":
                    if teleop_just_pressed and not self.emergency_stop_active and self.wall_is_detected:
                        self.transition_to("wall_following")
                case "estop":
                    if not self.emergency_stop_active and teleop_just_pressed:
                        self.transition_to("teleop")
                case _:
                    self.get_logger().warn(f"Unknown state: {self.current_state}")

            self.last_teleop_state = self.teleop_key_pressed

        self.broadcast_current_state()

    def transition_to(self, new_state):
        """Handle state transitions"""
        if new_state != self.current_state:
            self.previous_state = self.current_state
            self.current_state = new_state
            self.get_logger().info(f"State: {self.previous_state} -> {self.current_state}")

    def publish_active_command(self):
        """
        Publish appropriate velocity command based on current state.
        """
        if self.emergency_stop_active:
            self.send_stop_command()
        elif self.current_state == "obstacle_avoidance":
            self.robot_cmd_publisher.publish(self.latest_obstacle_cmd)
        elif self.current_state == "wall_following":
            self.robot_cmd_publisher.publish(self.latest_wall_cmd)
        elif self.current_state == "teleop":
            self.robot_cmd_publisher.publish(self.latest_teleop_cmd)
        else:
            # Safety fallback
            self.send_stop_command()
    
    def send_stop_command(self):
        """Send zero velocity to stop robot"""
        stop_cmd = Twist()  # All zeros by default
        self.robot_cmd_publisher.publish(stop_cmd)
    
    # Velocity command handlers
    def handle_obstacle_cmd(self, msg):
        """Store latest obstacle avoidance command"""
        self.latest_obstacle_cmd = msg
        
    def handle_wall_cmd(self, msg):
        """Store latest wall following command"""
        self.latest_wall_cmd = msg
        
    def handle_teleop_cmd(self, msg):
        """Store latest teleop command"""
        self.latest_teleop_cmd = msg

    # State condition callbacks
    def on_waypoint_reached(self, msg):
        with self.state_lock:
            self.waypoint_reached = bool(msg.data)
            if self.waypoint_reached:
                self.get_logger().info("Waypoint reached!")

    def on_wall_detected(self, msg):
        with self.state_lock:
            self.wall_is_detected = bool(msg.data)

    def on_estop_triggered(self, msg):
        with self.state_lock:
            self.emergency_stop_active = bool(msg.data)
            if self.emergency_stop_active:
                self.get_logger().warn("EMERGENCY STOP ACTIVATED!")

    def on_teleop_pressed(self, msg):
        with self.state_lock:
            self.teleop_key_pressed = bool(msg.data)

    def broadcast_current_state(self):
        """Publish current state to all behavior nodes"""
        state_msg = String()
        state_msg.data = self.current_state
        self.state_publisher.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FiniteStateController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()