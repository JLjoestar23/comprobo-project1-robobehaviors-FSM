import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from threading import Thread
import tty
import select
import sys
import termios

class Teleop(Node):
    """ This node's functionality allows the user to control a Neato with a keyboard."""
    def __init__(self):
        # intialize node name
        super().__init__('teleop')
        # create a publisher that will set linear and angular velocity
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # keylogging variables
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = None

        # velocity command variables
        self.velocity = Twist()
        self.x_vel = 0.0
        self.z_angular_vel = 0.0

        # initiate a blocking, timer-based loop
        self.running = True
        self.key_thread = Thread(target=self.teleop_loop())
        self.key_thread.start()


    def get_key(self):
        """Receive, process, and return the detected keystroke."""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def teleop_loop(self):
        """Main loop that allows drive control depending with the keyboard."""
        while self.running:
            # get the key stroke
            self.key = self.get_key()
            print(self.key)

            if self.key == "w": # increment linear velocity forward
                self.velocity.linear.x = round(self.velocity.linear.x + 0.1, 1)
            elif self.key == "s": # increment linear velocity reverse
                self.velocity.linear.x = round(self.velocity.linear.x - 0.1, 1)
            elif self.key == "a": # increment angular velocity left
                self.velocity.angular.z = round(self.velocity.angular.z + 0.5, 1)
            elif self.key == "d": # increment angular velocity right
                self.velocity.angular.z = round(self.velocity.angular.z - 0.5, 1)
            elif self.key == "e": # angular velocity stop
                self.velocity.angular.z = 0.0
            elif self.key == "q": # linear velocity stop
                self.velocity.linear.x = 0.0
            elif self.key == " ": # both stop
                self.velocity.linear.x = 0.0
                self.velocity.angular.z = 0.0
            elif self.key == "!": # shutdown the node
                self.velocity.linear.x = 0.0
                self.velocity.angular.z = 0.0
                self.publisher.publish(self.velocity)
                self.running = False
                rclpy.shutdown()

            # publish velocity commands
            self.publisher.publish(self.velocity)
            print("Linear Velocity: " + str(self.velocity.linear.x))
            print("Angular Velocity: " + str(self.velocity.angular.z))


def main(args=None):
    rclpy.init(args=args)
    node = Teleop()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()