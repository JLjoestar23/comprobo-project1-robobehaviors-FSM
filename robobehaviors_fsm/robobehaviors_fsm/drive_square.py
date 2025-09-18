import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistWithCovariance, Pose, PoseWithCovariance
from nav_msgs.msg import Odometry
from threading import Thread
import math

class DriveSquare(Node):
    """ This node's functionality is to drive a Neato to a targeted distance away from a detected obstacle."""
    def __init__(self):
        # intialize node name
        super().__init__('drive_square')
        # create a publisher that will set linear and angular velocity
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        # create a subscriber that subscribes to /odom
        self.odom = self.create_subscription(Odometry, "/odom", self.process_pose, 10)

        # velocity command variables
        self.velocity = Twist()
        self.x_vel = 0.0
        self.z_angular_vel = 0.0

        # controller variables
        self.desired_x = 0.0
        self.desired_heading = 45.0
        self.current_x = 0.0
        self.current_heading = 0.0

        # quaternion related
        self.q = None

        # initiate a blocking, timer-based loop
        self.running = True
        self.main_thread = Thread(target=self.main_loop)
        self.main_thread.start()
    
    def main_loop(self):
        while self.running:
            self.velocity.angular.z = -0.05 * (self.current_heading - self.desired_heading)
            self.publisher.publish(self.velocity)


    def process_pose(self, data):
        self.current_x = data.pose.pose.position.x
        self.q = data.pose.pose.orientation
        self.current_heading = self.euler_yaw_from_quat(self.q) * (180/math.pi)
        #print(self.current_x)
        print(self.current_heading)

    def euler_yaw_from_quat(self, q):
        return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))


def main(args=None):
    rclpy.init(args=args)
    node = DriveSquare()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()