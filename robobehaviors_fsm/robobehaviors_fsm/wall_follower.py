import math
import numpy as np
from sklearn.linear_model import LinearRegression

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


def normalize_angle_to_pi(angle):
    """Wrap angle to stay within -pi and pi range"""
    return math.atan2(math.sin(angle), math.cos(angle))


class WallFollowerNode(Node):
    """
    Wall following behavior using LIDAR-based line detection
    """
    
    def __init__(self):
        super().__init__('wall_follower')
        
        # wall following parameters
        self.target_wall_distance = 0.1
        self.distance_gain = 1.0
        self.angle_gain = 2.0
        self.base_speed = 0.3

        # Lidar processing window
        self.scan_window_angle = math.radians(40)
        self.scan_direction = 0.0

        # State tracking
        self.robot_fsm_state = None

        # Ros subscriptions and publishers
        self.create_subscription(LaserScan, '/scan', self.process_lidar_scan, 10)
        self.create_subscription(String, '/current_state', self.update_fsm_state, 10)

        self.velocity_publisher = self.create_publisher(Twist, '/wall_following_cmd', 10)
        self.wall_detection_publisher = self.create_publisher(Bool, 'wall_detected', 10)

        self.get_logger().info('Wall follower node initialized')

    def update_fsm_state(self, state_msg):
        """Update current FSM state"""
        self.robot_fsm_state = state_msg.data

    def process_lidar_scan(self, scan_msg):
        """
        main lidar processing
        """
        wall_found = self.is_wall_detected(scan_msg)

        # Report wall detection status to fsm
        detection_msg = Bool()
        detection_msg.data = wall_found
        self.wall_detection_publisher.publish(detection_msg)

        # only control robot if in wall_following state
        if self.robot_fsm_state == "wall_following" and wall_found:
            self.follow_detected_wall(scan_msg)

    def extract_scan_points(self, scan_msg, min_angle, max_angle):
        """
        Extract valid lidar points within specified angular range
        """
        x_points, y_points = [], []
        
        for i, range_value in enumerate(scan_msg.ranges):
            if range_value <= 0.0 or range_value == float('inf'):
                continue

            point_angle = scan_msg.angle_min + i * scan_msg.angle_increment - math.pi
            point_angle = normalize_angle_to_pi(point_angle)

            if min_angle <= point_angle <= max_angle:
                x_points.append(range_value * math.cos(point_angle))
                y_points.append(range_value * math.sin(point_angle))

        return x_points, y_points

    def fit_line_to_points(self, x_coords, y_coords):
        """
        fit regression line through lidar points
        """
        if len(x_coords) < 2:
            return None, None, None

        x_array = np.array(x_coords).reshape((-1, 1))
        y_array = np.array(y_coords)

        line_model = LinearRegression()
        line_model.fit(x_array, y_array)
        
        fit_quality = line_model.score(x_array, y_array)
        
        return float(line_model.coef_[0]), float(line_model.intercept_), float(fit_quality)

    def send_velocity_command(self, linear_speed, angular_speed):
        """Publish velocity command to robot"""
        velocity_cmd = Twist()
        velocity_cmd.linear.x = linear_speed
        velocity_cmd.angular.z = angular_speed
        self.velocity_publisher.publish(velocity_cmd)

    def follow_detected_wall(self, scan_msg):
        """
        Core wall following algorithm
        """
        window_min = self.scan_direction - self.scan_window_angle / 2
        window_max = self.scan_direction + self.scan_window_angle / 2
        
        x_coords, y_coords = self.extract_scan_points(scan_msg, window_min, window_max)

        slope, intercept, fit_quality = self.fit_line_to_points(x_coords, y_coords)
        
        if slope is not None:
            # Calculate distance and angle from wall
            wall_distance = abs(intercept) / math.sqrt(slope**2 + 1)
            wall_angle = math.atan(slope)

            avg_x = float(np.mean(x_coords))
            avg_y = float(np.mean(y_coords))
            bearing_to_wall = math.atan2(avg_y, avg_x)

            angle_difference = normalize_angle_to_pi(bearing_to_wall - wall_angle)
            self.scan_direction = normalize_angle_to_pi(
                wall_angle + math.copysign(math.pi/2, math.sin(angle_difference))
            )

            distance_error = self.target_wall_distance - wall_distance
            angle_error = wall_angle

            angular_velocity = self.distance_gain * distance_error + self.angle_gain * angle_error

            self.send_velocity_command(self.base_speed, angular_velocity)
        else:
            # No wall detected - reset scan direction
            self.scan_direction = 0.0
            self.send_velocity_command(self.base_speed, 0.0)

    def is_wall_detected(self, scan_msg, quality_threshold=0.5):
        """
        check if a wall is present in front of robot
        """
        forward_min_angle = -math.pi/4
        forward_max_angle = math.pi/4
        
        x_coords, y_coords = self.extract_scan_points(scan_msg, forward_min_angle, forward_max_angle)
        _, _, fit_quality = self.fit_line_to_points(x_coords, y_coords)

        if fit_quality is not None:
            return fit_quality >= quality_threshold
        return False


def main(args=None):
    """
    Entry point for wall follower node
    """
    rclpy.init(args=args)
    
    try:
        wall_follower = WallFollowerNode()
        rclpy.spin(wall_follower)
    except KeyboardInterrupt:
        print("Wall follower node interrupted")
    finally:
        wall_follower.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()