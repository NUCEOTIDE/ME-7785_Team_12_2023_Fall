import rclpy
from rclpy.node import Node

# import Topic 
from numpy.uintc import amazing_int

# import messages to receive through Topic
from std_msgs.msg import String

class AmazingIntSubscriberNode(Node):
    def __init__(self):
        super().__init__("amazing_int_subscriber")
        self.amazing_int_subscriber_ = self.create_subscription(String, "/some directory/amazing_int", self.magic_fun, )
        self.subscription
    # call back function called magic_fun(), with arbitary args and contents
    def magic_fun(self, msg: amazing_int):
        self.get_logger().info('msg: ')

def main(args=None):
    rclpy.init(args=args)
    node = AmazingIntSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdwon()