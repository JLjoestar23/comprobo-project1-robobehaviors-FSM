import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def wrap_angle(line_angle: float) -> float:
    # wrap an angle to be within 0-360 bounds
    return math.atan2(math.sin(line_angle), math.cos(line_angle))


class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower')

        # control parameters, constants for distance and angle
        self.desired_distance = 0.3
        self.k_d = 1.0
        self.k_a = 2.0
        self.v0 = 0.3

        # lidar window to look for the wall
        self.window_width = math.radians(40)
        self.window_center = 0.0

        # ros things
        self.create_subscription(
            LaserScan,
            'scan',
            self._process_scan,
            qos_profile=qos_profile_sensor_data
        )
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)


    def _process_scan(self, msg: LaserScan):
        # callback for lidar data
        self._detect_and_follow_wall(msg)

    def _detect_and_follow_wall(self, msg: LaserScan):
        # extract points in the current window
        x_coords, y_coords = self._extract_points_in_window(msg)

        # if enough points to make a line
        if len(x_coords) > 2:
            # fit to wall points to get slope
            m, b = np.polyfit(x_coords, y_coords, 1)

            # perpendicular distance from line to robot
            dist = abs(b) / math.sqrt(m**2 + 1)
            line_angle = math.atan(m)

            
            # get the general cluster of the line, and the bearing toward it
            mean_x, mean_y = float(np.mean(x_coords)), float(np.mean(y_coords))
            bearing = math.atan2(mean_y, mean_x)

            # changes the desired center to be perpendicular to the line
            # uses the bearing to determine what direction to rotate the window
            diff = wrap_angle(bearing - line_angle)
            desired_center = wrap_angle(line_angle + math.copysign(math.pi/2, math.sin(diff)))
            self.window_center = desired_center

            # proportional control using both sistance to line and line angle
            e_dist = self.desired_distance - dist
            e_angle = line_angle
            omega = self.k_d * e_dist + self.k_a * e_angle

            self._publish_cmd(self.v0, omega)
        else:
            # if no wall reset window and speed
            self.window_center = 0.0
            self._publish_cmd(self.v0, 0.0)

    def _extract_points_in_window(self, msg: LaserScan):
        # get lidar points within the angular window
        x_coords, y_coords = [], []

        theta_min = self.window_center - self.window_width / 2
        theta_max = self.window_center + self.window_width / 2

        for i, r in enumerate(msg.ranges):
            if r <= 0.0 or r == float('inf'):
                continue

            # shift the lidar angles by 180 because it is shifted
            theta = msg.angle_min + i * msg.angle_increment - math.pi
            theta = wrap_angle(theta)

            # only add the data if angle is within the window
            if theta_min <= theta <= theta_max:
                x_coords.append(r * math.cos(theta))
                y_coords.append(r * math.sin(theta))

        return x_coords, y_coords

    def _publish_cmd(self, v: float, omega: float):
        # publish velocity command
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
