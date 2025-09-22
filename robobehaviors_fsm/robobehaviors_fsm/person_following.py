import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from visualization_msgs.msg import Marker
from threading import Thread
import math
import time

class PersonFollowing(Node):
    """
    A ROS2 Node that enables a Neato robot to follow a person or object by using LIDAR data
    to detect an obstacle in front and generate a waypoint a fixed distance away from it.
    The detected obstacle's position is transformed to the 'odom' frame and visualized in RViz.
    """
    def __init__(self):
        super().__init__('person_following')

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)  # for motion control
        self.marker_publisher = self.create_publisher(Marker, "/visualization_marker", 10)  # for RViz debugging

        # Subscribers
        self.odom = self.create_subscription(Odometry, "/odom", self.process_pose, 10)  # get current position
        self.subscriber = self.create_subscription(LaserScan, "/scan", self.process_scan, qos_profile=qos_profile_sensor_data)  # get LIDAR data

        # LIDAR scan data
        self.left_ranges = []     # distances on the left side
        self.left_rads = []       # angles on the left side
        self.right_ranges = []    # distances on the right side
        self.right_rads = []      # angles on the right side
        self.combined_ranges = [] # combined distances
        self.combined_rads = []   # combined angles
        self.mean_x = 0.0         # mean position in LIDAR frame
        self.mean_y = 0.0

        # TF for coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.neato_frame_point = PointStamped()  # point in base_laser_link frame
        self.odom_frame_point = PointStamped()   # transformed point in odom frame

        # Velocity
        self.velocity = Twist()
        self.x_vel = 0.0
        self.z_angular_vel = 0.0

        # Navigation
        self.target_x = 0.0
        self.target_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_heading = 0.0
        self.target_heading = 0.0
        self.heading_error = 0.0
        self.euclidian_distance = 0.0

        # Orientation (quaternion)
        self.q = None

        # Visualization for debugging
        self.marker = Marker()

        # Thread for main control loop
        self.running = True
        self.main_thread = Thread(target=self.main_loop)
        self.main_thread.start()

    
    def main_loop(self):
        """
        Main loop that deals with navigation and control to drive a Neato in a square.
        """
        print("Starting the person following node...")
        time.sleep(1)

        while self.running:

            self.publish_debug_waypoint()

            print(f"Target Waypoint: {[self.target_x, self.target_y]}") # debugging purposes

            # calculate euclidian distance between target and current position
            # once before starting combined driving phase (distance error)
            self.calc_euclidian_distance()

            # while the distance error is above the error threshold, use
            # proportional control to command both linear and angular
            # velocity in the direction of error
            # linear velocity is proportional to distance error, while
            # angular velocity is still proportional to heading error
            while (abs(self.euclidian_distance) > 0.75):
                self.calc_target_heading()
                self.calc_euclidian_distance()
                self.heading_error = self.normalize_angle(self.target_heading - self.current_heading)

                # if self.heading_error > 0.08:
                #     while abs(self.heading_error) > 0.005 and self.running:
                #         # Recalculate heading every loop
                #         self.calc_target_heading()
                #         self.heading_error = self.normalize_angle(self.target_heading - self.current_heading)

                #         # Command angular velocity
                #         self.velocity.linear.x = 0.0
                #         self.velocity.angular.z = self.heading_error
                #         self.cmd_vel_publisher.publish(self.velocity)

                self.velocity.angular.z = 1.5 * self.heading_error
                self.velocity.linear.x = min(0.3, 1.5 * self.euclidian_distance)
                self.cmd_vel_publisher.publish(self.velocity)
            
            # set all velocity to 0 once combined driving phase is finished
            self.velocity.linear.x = 0.0
            self.velocity.angular.z = 0.0
            self.cmd_vel_publisher.publish(self.velocity)
            

    # guidance and navigation related functions
    def process_pose(self, data):
        """
        Extract the robot's current x, y, and heading (yaw) from the /odom topic.
        """
        self.current_x = data.pose.pose.position.x
        self.current_y = data.pose.pose.position.y
        self.q = data.pose.pose.orientation
        self.current_heading = self.euler_yaw_from_quat(self.q)
        
        # for debugging
        #print(self.current_x)
        #print(self.current_heading)
        #print([self.target_x, self.target_y])

    def euler_yaw_from_quat(self, q):
        """
        Convert quaternion orientation to a 2D yaw value (rads).
        """
        return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def calc_target_heading(self):
        """
        Compute the angle (heading) from the robot's current position 
        to the target.
        """
        self.target_heading = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
        #print(self.target_heading) # for debugging

    def calc_euclidian_distance(self):
        """
        Compute the Euclidean distance between the robot and the target 
        position.
        """
        self.euclidian_distance = math.sqrt((self.target_x - self.current_x)**2 + (self.target_y - self.current_y)**2)
        #print(self.euclidian_distance) # for debugging

    def process_scan(self, scan):
        """
        Process LIDAR scan data to compute the mean position of valid points
        to potentially track that are within a centered, 90 degree FOV.
        """
        # Clear previous data
        self.left_ranges = []
        self.left_rads = []
        self.right_ranges = []
        self.right_rads = []

        ranges = scan.ranges  # full 360 deg LIDAR ranges list (len = 360)

        # process right of center LIDAR scan
        for i, r in enumerate(ranges[0:45]):
            if r is not None and r > 0.0 and not math.isinf(r):
                rads = math.radians(i)
                self.left_rads.append(rads)
                self.left_ranges.append(r)

        # process left of center LIDAR scan
        for i, r in enumerate(ranges[315:359]):
            if r is not None and r > 0.0 and not math.isinf(r):
                rads = math.radians(i + 315) # offset angle
                self.right_rads.append(rads)
                self.right_ranges.append(r)

        # combined ordered range and angle data into single list
        self.combined_ranges = self.left_ranges + self.right_ranges
        self.combined_rads = self.left_rads + self.right_rads

        result = self.compute_mean_cartesian(self.combined_ranges, self.combined_rads)

        print([self.target_x, self.target_y])

        if result is None:
            self.get_logger().warn("No valid LIDAR points to compute new mean.")
            return

        self.mean_x, self.mean_y = result

        #print([self.mean_x, self.mean_y])

        self.transform_laser_to_odom()

        self.target_x = self.odom_frame_point.point.x
        self.target_y = self.odom_frame_point.point.y

        #print([self.target_x, self.target_y])

        if math.isnan(self.mean_x) or math.isnan(self.mean_y):
            self.get_logger().warn("Mean scan point is NaN. Skipping transform.")
            return

    
    def compute_mean_cartesian(self, ranges, rads):
        """
        Convert polar coordinates (range, angle) to Cartesian (x, y),
        then compute the average (mean) x and y.

        Args:
            ranges (list): LIDAR ranges in meters
            rads (list): Corresponding angles in radians
 
        Returns:
            tuple: (mean_x, mean_y) in the base_laser_link frame
        """

        sum_x = 0.0
        sum_y = 0.0
        count = 0

        for i in range(len(ranges)):
            # Calculate the actual angle for this index
            angle_rad = rads[i]

            # Sum Cartesian components for averaging angle
            sum_x += ranges[i] * math.cos(angle_rad)
            sum_y += ranges[i] * math.sin(angle_rad)

            count += 1

        # return None if no valid LIDAR data is available
        if count == 0:
            return None

        # Mean angle by averaging x,y unit vectors and getting atan2
        mean_x = sum_x / count
        mean_y = sum_y / count

        return (mean_x, mean_y)

    def transform_laser_to_odom(self):
        """
        Transform the mean scan point from the 'base_laser_link' frame to the 'odom' frame.
        Saves the result in self.odom_frame_point.
        """

        self.neato_frame_point.header.stamp = self.get_clock().now().to_msg()
        self.neato_frame_point.header.frame_id = "base_laser_link"
        self.neato_frame_point.point.x = -self.mean_x
        self.neato_frame_point.point.y = -self.mean_y
        self.neato_frame_point.point.z = 0.0

        try:
            transform = self.tf_buffer.lookup_transform(
                "odom", "base_laser_link", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )
            self.odom_frame_point = do_transform_point(self.neato_frame_point, transform)

        except Exception as e:
            self.get_logger().warn(f"Transform failed: {e}")

    def publish_debug_waypoint(self):
        """
        Publish a green spherical marker at the current target position in the 'odom' frame.
        Useful for visualizing the detected obstacle location in RViz.
        """
        # setting header
        self.marker.header.frame_id = "odom"
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.ns = "my_namespace"
        self.marker.id = 0

        # marker properties
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = self.target_x
        self.marker.pose.position.y = self.target_y
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0

        # publish marker
        self.marker_publisher.publish(self.marker)

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowing()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()