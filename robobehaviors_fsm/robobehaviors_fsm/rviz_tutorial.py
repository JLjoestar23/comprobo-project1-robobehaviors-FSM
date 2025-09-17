import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class RvizTutorial(Node):
    """ This node's functionality is to drive a Neato to a targeted distance away from a detected obstacle."""
    def __init__(self):
        # intialize node name
        super().__init__('rviz_tutorial')
        # create a publisher that will set linear and angular velocity
        self.publisher = self.create_publisher(Marker, "/visualization_marker", 10)

        # initialize marker object
        self.marker = Marker()

        # initiate a blocking, timer-based loop
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_marker)

    def publish_marker(self):
        # setting header
        self.marker.header.frame_id = "odom"
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.ns = "my_namespace"
        self.marker.id = 0

        # marker properties
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
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
        self.publisher.publish(self.marker)


def main(args=None):
    rclpy.init(args=args)
    node = RvizTutorial()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()