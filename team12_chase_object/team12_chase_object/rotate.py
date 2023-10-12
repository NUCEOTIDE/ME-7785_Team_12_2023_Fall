import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point,Twist
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

class Rotate_Robot(Node):

    def __init__(self):
        # Creates the node.
        super().__init__('rotate_robot')

        self._coordinate_subscriber = self.create_subscription(Point,'object_coordinates',self._coordinates_callback,10)
        self._vel_publisher = self.create_publisher(Twist,'/cmd_vel',10)
        self.k = -2
    def _coordinates_callback(self,Point):
        coordinate = Point
        angular_velocity = coordinate.x * self.k
        velocity = Twist()
        velocity.linear.x = 0.0
        velocity.linear.y = 0.0
        velocity.linear.z = 0.0
        velocity.angular.x = 0.0
        velocity.angular.y = 0.0
        velocity.angular.z = angular_velocity
        if angular_velocity:
        	self.get_logger().info('Rotating velocity:' + str(angular_velocity) + 'rad/s')
        self._vel_publisher.publish(velocity)


def main():

    rclpy.init()  # init routine needed for ROS2.
    rotater = Rotate_Robot()

    rclpy.spin(rotater)
    # Clean up and shutdown.
    rotater.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



