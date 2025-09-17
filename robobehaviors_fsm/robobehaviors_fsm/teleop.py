import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from threading import Thread, Event
import time

class WallApproach(Node):
    """ This node's functionality is to drive a Neato to a targeted distance away from a detected obstacle."""
    def __init__(self):
        # intialize node name
        super().__init__('wall_approach')
        # create a subscription to the LIDAR data through the "/scan" topic
        self.subscriber = self.create_subscription(LaserScan, "/scan", self.process_scan, qos_profile=qos_profile_sensor_data)
        # create a publisher that will set linear and angular velocity
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # declaring params
        self.declare_parameter("target_distance", 0.5) # proportional constant for our controller
        self.declare_parameter("Kp", 0.5) # desired distance from obstacle

        # initialize variables
        self.vel_cmd = Twist() # initialize Twist object
        self.obstacle_dist = 0.0 # measured distance to an obstacle based on LIDAR data
        self.Kp = self.get_parameter("Kp").value
        self.target_dist = self.get_parameter("target_distance").value
        self.lin_vel_x = 0.0 # linear x velocity speeds

        # initiate a blocking, timer-based loop
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.wall_approach)

        # initiate create thread to handle main loop
        #self.wall_approach_thread = Thread(target=self.wall_approach)
        #self.wall_approach_thread.start()

        # the following line is only need to support dynamic_reconfigure
        self.add_on_set_parameters_callback(self.parameter_callback)

    def wall_approach(self):
        """Calculate and publish velocity commands based on distance to target."""
        # if invalid range, drive at constant velocity
        if self.obstacle_dist == None:
            self.vel_cmd.linear.x = 0.3
        else:
            # proportional control
            self.vel_cmd.linear.x = self.Kp * (self.obstacle_dist - self.target_dist)
        
        # bounded to physical limits of the Neato
        if self.vel_cmd.linear.x > 0.3:
            self.vel_cmd.linear.x = 0.3
        elif self.vel_cmd.linear.x < -0.3:
            self.vel_cmd.linear.x = -0.3
        
        # publish velocity commands
        self.publisher.publish(self.vel_cmd)
        
        
    def process_scan(self, scan):
        """Processes frontal LIDAR data."""
        if scan.ranges[0] != 0.0:
            # checking for the value 0.0 ensures the data is valid.
            self.obstacle_dist = scan.ranges[0]

    def parameter_callback(self, params):
        """ This callback allows the parameters of the node to be adjusted
            dynamically by other ROS nodes (e.g., dynamic_reconfigure).
            This function is only needed to support dynamic_reconfigure. """
        for param in params:
            if param.name == "Kp" and param.type_ == Parameter.Type.DOUBLE:
                self.Kp = param.value
            elif param.name == "target_distance" and param.type_ == Parameter.Type.DOUBLE:
                self.target_dist = param.value
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = WallApproach()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()