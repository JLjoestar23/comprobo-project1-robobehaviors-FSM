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
from std_msgs.msg import Bool, String
from threading import Thread
import math
import time

class ObstacleAvoidance(Node):
    """
    A ROS2 Node that enables a Neato robot to follow a person or object by using LIDAR data
    to detect an obstacle in front and generate a waypoint a fixed distance away from it.
    The detected obstacle's position is transformed to the 'odom' frame and visualized in RViz.
    """
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)  # for motion control
        self.marker_publisher = self.create_publisher(Marker, "/visualization_marker", 10)  # for RViz debugging
        self.target_reached_publisher = self.create_publisher(Bool, "/target_reached", 10) # publish if it has reached target waypoint

        # Subscribers
        self.odom = self.create_subscription(Odometry, "/odom", self.process_pose, 10)  # get current position
        self.scan_subscription = self.create_subscription(LaserScan, "/scan", self.potential_field_control, qos_profile=qos_profile_sensor_data)  # get LIDAR data
        self.create_subscription(String, "current_state", self.state_cb, 10)

        # Velocity
        self.velocity = Twist()
        self.x_vel = 0.0
        self.z_angular_vel = 0.0
        self.total_repulsion_lin_vel = 0.0
        self.total_repulsion_ang_vel = 0.0

        # Navigation
        self.target_x = 0.0
        self.target_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_heading = 0.0
        self.target_heading = 0.0
        self.heading_error = 0.0
        self.distance_to_goal = 0.0

        # Orientation (quaternion)
        self.q = None

        # Visualization for debugging
        self.marker = Marker()

        # state gating
        self.fsm_state = ""
        self.target_reached = Bool()
        self.target_reached.data = False

        # Thread for main control loop
        self.running = True
        self.main_thread = Thread(target=self.main_loop)
        self.main_thread.start()
        self.get_logger().info('ObstacleAvoidanceNode initialized.')

    
    def main_loop(self):
        """
        Main loop that deals with navigation and control to drive a Neato in a square.
        """
        #print("Starting the obstacle avoidance node...")
        #time.sleep(1)

        # set target waypoint
        self.target_x = 4.0
        self.target_y = 0.0

        while self.running:

            #self.get_logger().info(self.fsm_state)

            if self.fsm_state == "obstacle_avoidance":
                # control loop until a certain distance threshold is reached
                #self.get_logger().info(f"Inside if statement")
                if self.distance_to_goal >= 0.1:
                    self.target_reached.data = False
                    self.target_reached_publisher.publish(self.target_reached)
                    self.velocity.linear.x = self.total_repulsion_lin_vel
                    self.velocity.angular.z = self.total_repulsion_ang_vel
                    self.cmd_vel_publisher.publish(self.velocity)
                elif self.distance_to_goal < 0.1:
                    # once within a certain distance threshold, update target reached status
                    self.velocity.linear.x = 0.0
                    self.velocity.angular.z = 0.0
                    self.cmd_vel_publisher.publish(self.velocity)
                    self.target_reached.data = True
                    self.target_reached_publisher.publish(self.target_reached)


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
    
    def potential_field_control(self, scan):
        """
        Combines attractive and repulsive vector fields using LIDAR and target pose.
        Outputs total linear and angular velocity components to steer robot.
        """
        if self.fsm_state == "obstacle_avoidance":
            # Constants
            attraction_gain = 1.0
            repulsion_gain = 0.1
            influence_radius = 0.75  # meters
            min_obstacle_dist = 0.05  # avoid division by near-zero

            # Attractive force towards goal
            dx = self.target_x - self.current_x
            dy = self.target_y - self.current_y
            self.distance_to_goal = math.hypot(dx, dy)

            # Unit vector to goal
            if self.distance_to_goal > 0.1:
                fx_total = attraction_gain * dx / self.distance_to_goal
                fy_total = attraction_gain * dy / self.distance_to_goal
            else:
                fx_total = 0.0
                fy_total = 0.0

            # Repulsive forces from LIDAR
            angle = scan.angle_min
            for r in scan.ranges:
                if r == 0.0 or math.isinf(r) or r > influence_radius:
                    angle += scan.angle_increment
                    continue

                # Obstacle position in robot frame
                obs_x = r * math.cos(angle)
                obs_y = r * math.sin(angle)

                dist = math.hypot(obs_x, obs_y)
                if dist < min_obstacle_dist:
                    dist = min_obstacle_dist  # prevent division errors

                # Unit vector from obstacle to robot
                rep_x = obs_x / dist
                rep_y = obs_y / dist

                # Scale repulsion force (decays with distance)
                strength = repulsion_gain * (1.0 / dist - 1.0 / influence_radius)
                strength = max(0.0, strength)  # only repel if within influence
                fx_total += strength * rep_x
                fy_total += strength * rep_y

                angle += scan.angle_increment

            # Convert combined force to heading and speed
            angle_to_move = math.atan2(fy_total, fx_total)
            speed = min(0.3, math.hypot(fx_total, fy_total))

            # Convert angle difference to angular velocity
            heading_error = self.normalize_angle(angle_to_move - self.current_heading)
            angular_velocity = 1.5 * heading_error  # simple proportional controller

            # Store for use in main loop
            self.total_repulsion_lin_vel = speed
            self.total_repulsion_ang_vel = angular_velocity

            # Debug print
            #self.get_logger().info(f"Speed: {speed:.3f}, Angular: {angular_velocity:.3f}, Heading error: {heading_error:.3f}")

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

    def state_cb(self, msg):
        self.fsm_state = msg.data  # store fsm state
        #self.get_logger().info(self.fsm_state)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()