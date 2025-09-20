import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data

import math
import numpy as np
from sklearn.linear_model import LinearRegression

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_approach')
        self.create_subscription(
            LaserScan,
            'scan',
            self.process_scan,
            qos_profile=qos_profile_sensor_data
        )

    def run_loop(self):
        # not used for now
        pass

    def process_scan(self, msg):
        # if msg.ranges[0] != 0.0:
        #     distance = msg.ranges[0]
        #     theta = msg.angle_min
        #     #self.get_logger().info(f"Front distance: {distance:.2f} m")
        #     self.get_logger().info(f"Beam[0]: r = {distance:.2f} m, θ = {theta:.2f} rad")
        self.detect_wall(msg)



    # checks the 180 angle in front of the neato to see if there is a wall by doing linear regression
    # first step: get distances and angles for the 180 in front of us
    def detect_wall(self, msg):
        x_coords = []
        y_coords = []

        for i, r in enumerate(msg.ranges):
            if r == 0.0 or r == float('inf'):
                continue

            theta = msg.angle_min + i * msg.angle_increment
            

            # restrict to front 90 deg
            if -math.pi/6 <= theta <= math.pi/6:
                x_coords.append(r * math.cos(theta))
                y_coords.append(r * math.sin(theta))
                
        

        # only fit if we have enough points
        if len(x_coords) > 2:
            x = np.array(x_coords).reshape((-1, 1))
            y = np.array(y_coords)

            model = LinearRegression()
            model.fit(x, y)
            r_sq = model.score(x, y)
            self.get_logger().info(
                f"Line fit: y = {model.coef_[0]:.2f}x + {model.intercept_:.2f}, R² = {r_sq:.3f}"
            )


            


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
