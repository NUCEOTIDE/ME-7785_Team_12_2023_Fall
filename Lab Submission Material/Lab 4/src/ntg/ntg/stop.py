import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist



class Stoper(Node):
    def __init__(self):
        super().__init__('stoper')
        self.__vel_publisher = self.create_publisher(Twist, '/cmd_vel',10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        self.__vel_publisher.publish(msg)

def main():
    rclpy.init()
    change_parameter_node = Stoper()
    rclpy.spin(change_parameter_node)
    change_parameter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
