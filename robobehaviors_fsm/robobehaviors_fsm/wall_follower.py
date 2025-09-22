import math
import numpy as np
from sklearn.linear_model import LinearRegression

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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
        self.wall_pub = self.create_publisher(Bool, 'wall_detected', 10)

        self.get_logger().info('WallFollowerNode initialized.')

    def _state_cb(self, msg: String):
        self.get_logger().info("Entered _state_cb")
        """Update the FSM state"""
        self.current_state = msg.data
        self.get_logger().info(f"Current FSM state: {self.current_state}")

    def _process_scan(self, msg: LaserScan):
        self.get_logger().info("Entered _process_scan")

        # only follow walls if FSM says so
        if self.current_state != "wall_following":
            self.get_logger().info(f"State {self.current_state}, Not in wall_following state, skipping control.")
            return

        self._detect_and_follow_wall(msg)

        detected = self.detect_wall(msg)
        wall_msg = Bool()
        wall_msg.data = detected

        self.get_logger().info(f"Wall detected: {detected}")
        self.wall_pub.publish(wall_msg)

    def _extract_points(self, msg: LaserScan, theta_min: float, theta_max: float):
        self.get_logger().info("Entered _extract_points")

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
        self.get_logger().info("Entered _fit_line")

        if len(x_coords) < 2:
            return None, None, None

        x = np.array(x_coords).reshape((-1, 1))
        y = np.array(y_coords)

        model = LinearRegression()
        model.fit(x, y)
        r_sq = model.score(x, y)
        return float(model.coef_[0]), float(model.intercept_), float(r_sq)

    def _publish_cmd(self, v: float, omega: float):
        self.get_logger().info("Entered _publish_cmd")

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)

    def _detect_and_follow_wall(self, msg: LaserScan):
        self.get_logger().info("Entered _detect_and_follow_wall")

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

    def detect_wall(self, msg: LaserScan, r2_threshold: float = 0.6) -> bool:
        self.get_logger().info("Entered detect_wall")

        theta_min, theta_max = -math.pi/4, math.pi/4
        x_coords, y_coords = self._extract_points(msg, theta_min, theta_max)
        m, b, r_sq = self._fit_line(x_coords, y_coords)

        if r_sq is not None:
            self.get_logger().debug(f"Wall detection R^2 = {r_sq:.2f}")
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
