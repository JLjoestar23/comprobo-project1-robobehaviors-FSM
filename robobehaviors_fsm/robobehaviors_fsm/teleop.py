import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from threading import Thread, Event
import sys
import termios
import atexit


class TeleopController(Node):
    """
    Handles keyboard input for robot control.
    """
    
    def __init__(self):
        super().__init__('teleop_controller')
        
        # velocity and mode publishers - send commands to FSM, not directly to robot
        self.velocity_pub = self.create_publisher(Twist, "/teleop_cmd", 10)
        self.mode_toggle_pub = self.create_publisher(Bool, "teleop_toggle", 10)

        # Subscribe to FSM state updates
        self.create_subscription(String, "current_state", self.update_robot_state, 10)

        # Terminal setup for raw key input
        self.original_terminal_settings = termios.tcgetattr(sys.stdin)
        atexit.register(self.restore_terminal)

        # Current velocities
        self.current_velocity = Twist()
        self.linear_speed = 0.0
        self.angular_speed = 0.0

        # State tracking
        self.robot_state = "unknown"
        self.teleop_mode_active = False
        
        # Control flags
        self.keep_running = True
        self.shutdown_requested = Event()

        # Setup terminal and start keyboard thread
        self.configure_terminal_for_raw_input()
        self.keyboard_thread = Thread(target=self.keyboard_handler, daemon=True)
        self.keyboard_thread.start()
        
        self.print_startup_message()
        
        self.get_logger().info("Teleop node initialized")


    def configure_terminal_for_raw_input(self):
        """Configure terminal to read keys immediately without buffering"""
        try:
            new_settings = termios.tcgetattr(sys.stdin)
            # Disable canonical mode (line buffering) and echo
            new_settings[3] = new_settings[3] & ~(termios.ECHO | termios.ICANON)
            new_settings[6][termios.VMIN] = 1  # Read minimum 1 character
            new_settings[6][termios.VTIME] = 0  # No timeout
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, new_settings)
        except Exception as e:
            self.get_logger().error(f"Failed to configure terminal: {e}")

    def restore_terminal(self):
        """Restore original terminal settings on exit"""
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.original_terminal_settings)
        except:
            pass  # Don't crash on exit

    def print_startup_message(self):
        """Print helpful startup information"""
        print("\n" + "="*50)
        print("  ROBOT TELEOP CONTROLLER")
        print("="*50)
        print("Controls:")
        print("  t     - Toggle teleop mode")
        print("  w/s   - Forward/backward")  
        print("  a/d   - Turn left/right")
        print("  space - Stop all movement")
        print("  q     - Quit teleop")
        print("\nTerminal is in raw mode - keys work immediately!")
        print("Current state: Waiting for FSM connection...")
        print("-"*50)

    def update_robot_state(self, msg):
        """Update current robot state from FSM"""
        self.robot_state = msg.data

    def read_single_keypress(self):
        """Read one character from stdin immediately"""
        try:
            return sys.stdin.read(1)
        except:
            return None

    def keyboard_handler(self):
        """Main keyboard input loop - runs in separate thread"""
        while self.keep_running and not self.shutdown_requested.is_set():
            try:
                key = self.read_single_keypress()
                if key:
                    self.process_keypress(key)
            except KeyboardInterrupt:
                self.keep_running = False
                break
            except Exception as e:
                self.get_logger().debug(f"Key read error: {e}")
                continue

    def process_keypress(self, key):
        """Handle individual key presses"""
        # Always show what key was pressed and current state
        print(f"Key: '{key}' | Robot: {self.robot_state:12s}", end=" | ")
        
        # Toggle teleop mode (works in any state except estop for exit)
        if key == "t":
            self.teleop_mode_active = not self.teleop_mode_active
            toggle_msg = Bool()
            toggle_msg.data = self.teleop_mode_active
            self.mode_toggle_pub.publish(toggle_msg)
            
            status = "ENABLED" if self.teleop_mode_active else "DISABLED"
            print(f"Teleop: {status}")
            return

        # Movement commands only work in teleop state
        if self.robot_state == "teleop":
            self.handle_movement_command(key)
        else:
            print("(Not in teleop mode)")

    def handle_movement_command(self, key):
        """Process movement keys when in teleop mode"""
        movement_made = True

        match key:
            case "w":
                self.linear_speed = min(1.0, self.linear_speed + 0.1)
            case "s":
                self.linear_speed = max(-1.0, self.linear_speed - 0.1)
            case "a":
                self.angular_speed = min(2.0, self.angular_speed + 0.2)
            case "d":
                self.angular_speed = max(-2.0, self.angular_speed - 0.2)
            case " ":
                self.linear_speed = 0.0
                self.angular_speed = 0.0
            case "q":
                self.linear_speed = 0.0
                self.angular_speed = 0.0
                self.send_velocity_command()
                print("Shutting down teleop...")
                self.keep_running = False
                self.shutdown_requested.set()
                return
            case _:
                movement_made = False

        if movement_made:
            self.linear_speed = round(self.linear_speed, 2)
            self.angular_speed = round(self.angular_speed, 2)
            self.send_velocity_command()
            print(f"Speed: {self.linear_speed:+0.2f} m/s | Turn: {self.angular_speed:+0.2f} rad/s")
        else:
            print("(Unknown key)")


    def send_velocity_command(self):
        """Publish current velocity to robot"""
        self.current_velocity.linear.x = float(self.linear_speed)
        self.current_velocity.angular.z = float(self.angular_speed)
        self.velocity_pub.publish(self.current_velocity)


def main(args=None):
    """
    Main entry point for teleop controller
    """
    rclpy.init(args=args)
    
    try:
        teleop_node = TeleopController()
        
        # Keep ROS spinning while keyboard thread handles input
        while rclpy.ok() and teleop_node.keep_running:
            rclpy.spin_once(teleop_node, timeout_sec=0.1)
        
        # Clean shutdown
        teleop_node.destroy_node()
        
    except KeyboardInterrupt:
        print("\nShutdown requested...")
    except Exception as e:
        print(f"Error in teleop: {e}")
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()