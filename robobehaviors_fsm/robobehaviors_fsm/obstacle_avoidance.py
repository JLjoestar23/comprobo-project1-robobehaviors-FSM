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
    A ROS2 Node that enables a Neato robot to follow a person or object by using LIDAR data to detect an obstacle in front and generate a waypoint a fixed distance away from it.
    The detected obstacle's position is transformed to the 'odom' frame and visualized in RViz.

    Robot navigation using potential fields approach.
    """
    
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # velocity and other command publishers
        self.velocity_publisher = self.create_publisher(Twist, "/obstacle_avoidance_cmd", 10)
        self.marker_publisher = self.create_publisher(Marker, "/visualization_marker", 10)
        self.target_reached_publisher = self.create_publisher(Bool, "/target_reached", 10)

        # State and sensor subscriptions
        self.odometry_sub = self.create_subscription(Odometry, "/odom", self.update_robot_position, 10)
        self.laser_sub = self.create_subscription(LaserScan, "/scan", self.navigate_with_potential_fields, qos_profile=qos_profile_sensor_data)
        self.create_subscription(String, "current_state", self.update_fsm_state, 10)

        # Navigation command
        self.movement_command = Twist()
        
        # Current robot state
        self.robot_x_position = 0.0
        self.robot_y_position = 0.0
        self.robot_heading = 0.0
        
        # Target waypoint
        self.waypoint_x = 0.0
        self.waypoint_y = 0.0
        self.desired_heading = 0.0
        self.heading_error = 0.0
        self.distance_to_target = 0.0

        # Velocity commands from potential field algorithm
        self.forward_velocity = 0.0
        self.turning_velocity = 0.0

        # Robot orientation (quaternion from odometry)  
        self.orientation_quat = None

        # Debugging visualization
        self.debug_marker = Marker()

        # FSM integration
        self.current_fsm_state = ""
        self.goal_reached_status = Bool()
        self.goal_reached_status.data = False

        # Navigation control thread
        self.node_active = True
        self.navigation_thread = Thread(target=self.navigation_control_loop)
        self.navigation_thread.start()
        
        self.get_logger().info('Obstacle avoidance Node Initialized')

    def navigation_control_loop(self):
        """
        Main navigation loop - drives robot toward waypoint while avoiding obstacles
        """
        # Initial waypoint, 4 meters in front
        self.waypoint_x = 4.0
        self.waypoint_y = 0.0

        while self.node_active:
            if self.current_fsm_state == "obstacle_avoidance":
                # Keep moving toward target if not close enough
                if self.distance_to_target >= 0.1:
                    self.goal_reached_status.data = False
                    self.target_reached_publisher.publish(self.goal_reached_status)
                    
                    # Publish velocity commands calculated by potential field
                    self.movement_command.linear.x = self.forward_velocity
                    self.movement_command.angular.z = self.turning_velocity
                    self.velocity_publisher.publish(self.movement_command)
                    
                else:
                    # Target reached, stop and signal completion
                    self.movement_command.linear.x = 0.0
                    self.movement_command.angular.z = 0.0
                    self.velocity_publisher.publish(self.movement_command)
                    
                    self.goal_reached_status.data = True
                    self.target_reached_publisher.publish(self.goal_reached_status)

    def update_robot_position(self, odometry_msg):
        """
        Process odometry data to track current robot position and heading
        
        This is where we calculate distance to target since odometry
        updates are most frequent and reliable.
        """
        self.robot_x_position = odometry_msg.pose.pose.position.x
        self.robot_y_position = odometry_msg.pose.pose.position.y
        self.orientation_quat = odometry_msg.pose.pose.orientation
        self.robot_heading = self.convert_quaternion_to_yaw(self.orientation_quat)
        
        # Calculate distance to current waypoint
        dx = self.waypoint_x - self.robot_x_position
        dy = self.waypoint_y - self.robot_y_position
        self.distance_to_target = math.hypot(dx, dy)

    def convert_quaternion_to_yaw(self, quat):
        """Convert quaternion to 2D yaw angle in radians"""
        return math.atan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y), 
            1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        )
    
    def normalize_angle(self, angle):
        """Keep angle within -π to π range"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def navigate_with_potential_fields(self, laser_scan):
        """
        Core navigation algorithm using attractive and repulsive forces
        """
        if self.current_fsm_state != "obstacle_avoidance":
            return
            
        # Tuning parameters for navigation behavior
        attraction_strength = 1.0
        repulsion_strength = 0.1
        obstacle_influence_range = 0.75
        min_safe_distance = 0.05

        # Calculate attractive force toward waypoint
        displacement_x = self.waypoint_x - self.robot_x_position
        displacement_y = self.waypoint_y - self.robot_y_position
        distance_to_waypoint = math.hypot(displacement_x, displacement_y)

        # Create unit vector toward goal
        if distance_to_waypoint > 0.1:
            force_x = attraction_strength * displacement_x / distance_to_waypoint
            force_y = attraction_strength * displacement_y / distance_to_waypoint
        else:
            # Already at goal - no attractive force needed
            force_x = 0.0
            force_y = 0.0

        # Add repulsive forces from obstacles detected by LIDAR
        scan_angle = laser_scan.angle_min
        for range_reading in laser_scan.ranges:
            # Skip invalid or distant readings
            if (range_reading == 0.0 or math.isinf(range_reading) or 
                range_reading > obstacle_influence_range):
                scan_angle += laser_scan.angle_increment
                continue

            # Convert polar coordinates to cartesian (robot frame)
            obstacle_x = range_reading * math.cos(scan_angle)
            obstacle_y = range_reading * math.sin(scan_angle)

            obstacle_distance = math.hypot(obstacle_x, obstacle_y)
            if obstacle_distance < min_safe_distance:
                obstacle_distance = min_safe_distance

            # Create repulsive force pointing away from obstacle  
            repulsion_x = -obstacle_x / obstacle_distance
            repulsion_y = -obstacle_y / obstacle_distance

            # Scale force based on proximity (closer = stronger repulsion)
            force_magnitude = repulsion_strength * (1.0 / obstacle_distance - 1.0 / obstacle_influence_range)
            force_magnitude = max(0.0, force_magnitude)
            
            force_x += force_magnitude * repulsion_x
            force_y += force_magnitude * repulsion_y

            scan_angle += laser_scan.angle_increment

        # Convert combined force vector to robot commands
        desired_heading = math.atan2(force_y, force_x)
        desired_speed = min(0.3, math.hypot(force_x, force_y)) 

        # Calculate heading error and convert to angular velocity
        heading_error = self.normalize_angle(desired_heading - self.robot_heading)
        angular_velocity = 1.5 * heading_error

        # Store velocities for main control loop to publish
        self.forward_velocity = desired_speed
        self.turning_velocity = angular_velocity

    def create_waypoint_marker(self):
        """
        Create visualization marker for RViz debugging
        """
        # Set up marker properties
        self.debug_marker.header.frame_id = "odom"
        self.debug_marker.header.stamp = self.get_clock().now().to_msg()
        self.debug_marker.ns = "navigation_markers"
        self.debug_marker.id = 0

        # Marker appearance
        self.debug_marker.type = Marker.SPHERE
        self.debug_marker.action = Marker.ADD
        
        # Position at current waypoint
        self.debug_marker.pose.position.x = self.waypoint_x
        self.debug_marker.pose.position.y = self.waypoint_y
        self.debug_marker.pose.position.z = 0.0
        
        self.debug_marker.pose.orientation.x = 0.0
        self.debug_marker.pose.orientation.y = 0.0
        self.debug_marker.pose.orientation.z = 0.0
        self.debug_marker.pose.orientation.w = 1.0
        
        # Size and color
        self.debug_marker.scale.x = 0.1
        self.debug_marker.scale.y = 0.1
        self.debug_marker.scale.z = 0.1
        self.debug_marker.color.a = 1.0
        self.debug_marker.color.r = 0.0
        self.debug_marker.color.g = 1.0
        self.debug_marker.color.b = 0.0

        self.marker_publisher.publish(self.debug_marker)

    def update_fsm_state(self, state_msg):
        """Update current FSM state from finite state machine"""
        self.current_fsm_state = state_msg.data


def main(args=None):
    """
    Entry point for obstacle avoidance node
    """
    rclpy.init(args=args)
    
    try:
        navigation_node = ObstacleAvoidance()
        rclpy.spin(navigation_node)
    except KeyboardInterrupt:
        print("Obstacle avoidance node interrupted")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()