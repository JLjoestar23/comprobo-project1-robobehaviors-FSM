import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistWithCovariance, Pose, PoseWithCovariance
from nav_msgs.msg import Odometry
from threading import Thread
import math
import time

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
        self.x_waypoint = [1.0, 1.0, 0.0, 0.0]
        self.y_waypoint = [0.0, 1.0, 1.0, 0.0]
        self.target_x = 0.0
        self.target_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.target_heading = 0.0
        self.current_heading = 0.0
        self.heading_error = 0.0
        self.euclidian_distance = 0.0

        # quaternion related
        self.q = None

        # initiate a blocking, timer-based loop
        self.running = True
        self.main_thread = Thread(target=self.main_loop)
        self.main_thread.start()
    
    def main_loop(self):
        """Main loop that deals with navigation and control to drive a Neato in a square."""
        
        # loop through 4 sets of (x, y) waypoints
        for i in range(4):
            self.target_x = self.x_waypoint[i]
            self.target_y = self.y_waypoint[i]

            #print([self.target_x, self.target_y]) # debugging purposes

            # calculate target heading for the turn phase
            self.calc_target_heading()
            
            # calculate the heading error once before starting the turn phase
            self.heading_error = self.normalize_angle(self.target_heading - self.current_heading)

            #print(self.heading_error) #debugging purposes

            # while heading error is larger than error threshold of 0.008 rads,
            # use proportional control to command angular velocity in the
            # direction of error
            while (abs(self.heading_error) > 0.008):
                self.heading_error = self.normalize_angle(self.target_heading - self.current_heading)
                self.velocity.angular.z = 1.5 * self.heading_error
                self.publisher.publish(self.velocity)

            # stop command 0 angular velocity after the turn phase is complete
            self.velocity.angular.z = 0.0
            self.publisher.publish(self.velocity)
            
            time.sleep(1.0)

            # calculate euclidian distance between target and current position
            # once before starting combined driving phase (distance error)
            self.calc_euclidian_distance()

            # while the distance error is above the error threshold, use
            # proportional control to command both linear and angular
            # velocity in the direction of error
            # linear velocity is proportional to distance error, while
            # angular velocity is still proportional to heading error
            while (abs(self.euclidian_distance) > 0.05):
                self.calc_target_heading()
                self.calc_euclidian_distance()
                self.heading_error = self.normalize_angle(self.target_heading - self.current_heading)
                self.velocity.linear.x = 0.5 * self.euclidian_distance
                self.velocity.angular.z = 0.5 * self.heading_error
                self.publisher.publish(self.velocity)
            
            # set all velocity to 0 once combined driving phase is finished
            self.velocity.linear.x = 0.0
            self.velocity.angular.z = 0.0
            self.publisher.publish(self.velocity)


    def process_pose(self, data):
        """Process x position (m) and heading (deg) from the /odom topic."""
        self.current_x = data.pose.pose.position.x
        self.current_y = data.pose.pose.position.y
        self.q = data.pose.pose.orientation
        self.current_heading = self.euler_yaw_from_quat(self.q)
        # for debugging
        #print(self.current_x)
        #print(self.current_heading)
        #print([self.target_x, self.target_y])

    def euler_yaw_from_quat(self, q):
        """Convert quaternion orientation into a yaw value."""
        return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    
    def calc_target_heading(self):
        self.target_heading = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
        #print(self.target_heading) # for debugging

    def calc_euclidian_distance(self):
        self.euclidian_distance = math.sqrt((self.target_x - self.current_x)**2 + (self.target_y - self.current_y)**2)
        #print(self.euclidian_distance) # for debugging


def main(args=None):
    rclpy.init(args=args)
    node = DriveSquare()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()