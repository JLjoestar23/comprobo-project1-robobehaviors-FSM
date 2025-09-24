import math
import numpy as np
from sklearn.linear_model import LinearRegression

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


def wrap_angle(line_angle: float) -> float:
    return math.atan2(math.sin(line_angle), math.cos(line_angle))


class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower')
        print("Entered __init__")

        # control parameters
        self.desired_distance = 0.1
        self.k_d = 1.0
        self.k_a = 2.0
        self.v0 = 0.3

        # lidar window for following
        self.window_width = math.radians(40)
        self.window_center = 0.0

        # track current FSM state
        self.current_state = None

        # ros things
        self.create_subscription(
            LaserScan,
            '/scan', 
            self._process_scan,
            10
        )
        self.create_subscription(
            String,
            '/current_state',
            self._state_cb,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.wall_detected = self.create_publisher(Bool, 'wall_detected', 10)

        self.get_logger().info('WallFollowerNode initialized.')

    def _state_cb(self, msg: String):
        self.current_state = msg.data

    def _process_scan(self, msg: LaserScan):
        """Always check for wall detection, but only follow if FSM says so"""
        detected = self.detect_wall(msg)

        # Always publish detection
        wall_msg = Bool()
        wall_msg.data = detected
        self.wall_detected.publish(wall_msg)

        # Only control if state is wall_following
        if self.current_state == "wall_following" and detected:
            self._detect_and_follow_wall(msg)

    def _extract_points(self, msg: LaserScan, theta_min: float, theta_max: float):
        x_coords, y_coords = [], []
        for i, r in enumerate(msg.ranges):
            if r <= 0.0 or r == float('inf'):
                continue

            theta = msg.angle_min + i * msg.angle_increment - math.pi
            theta = wrap_angle(theta)

            if theta_min <= theta <= theta_max:
                x_coords.append(r * math.cos(theta))
                y_coords.append(r * math.sin(theta))

        return x_coords, y_coords

    def _fit_line(self, x_coords, y_coords):
        if len(x_coords) < 2:
            return None, None, None

        x = np.array(x_coords).reshape((-1, 1))
        y = np.array(y_coords)

        model = LinearRegression()
        model.fit(x, y)
        r_sq = model.score(x, y)
        return float(model.coef_[0]), float(model.intercept_), float(r_sq)

    def _publish_cmd(self, v: float, omega: float):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)

    def _detect_and_follow_wall(self, msg: LaserScan):
        theta_min = self.window_center - self.window_width / 2
        theta_max = self.window_center + self.window_width / 2
        x_coords, y_coords = self._extract_points(msg, theta_min, theta_max)

        m, b, r_sq = self._fit_line(x_coords, y_coords)
        if m is not None:
            dist = abs(b) / math.sqrt(m**2 + 1)
            line_angle = math.atan(m)

            mean_x, mean_y = float(np.mean(x_coords)), float(np.mean(y_coords))
            bearing = math.atan2(mean_y, mean_x)

            diff = wrap_angle(bearing - line_angle)
            self.window_center = wrap_angle(line_angle + math.copysign(math.pi/2, math.sin(diff)))

            e_dist = self.desired_distance - dist
            e_angle = line_angle
            omega = self.k_d * e_dist + self.k_a * e_angle

            self._publish_cmd(self.v0, omega)
        else:
            self.window_center = 0.0
            self._publish_cmd(self.v0, 0.0)

    def detect_wall(self, msg: LaserScan, r2_threshold: float = 0.5) -> bool:
        theta_min, theta_max = -math.pi/4, math.pi/4
        x_coords, y_coords = self._extract_points(msg, theta_min, theta_max)
        m, b, r_sq = self._fit_line(x_coords, y_coords)

        #self.get_logger().info(f"R2 = {r_sq}")
        if r_sq is not None:
            return r_sq >= r2_threshold
        return False


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
