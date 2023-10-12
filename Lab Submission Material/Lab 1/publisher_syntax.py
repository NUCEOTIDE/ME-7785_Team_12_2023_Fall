import rclpy
from rclpy.node import Node

# import Topic
from numpy.bool_ import amazing_bool

# import messsage to publish through Topic
from std_msgs.msg import String

class AmazingBoolPulisherNode(Node):
    def __init__(self):
        super().__init__("amazing_bool_publisher")
        self.publisher_=self.create_publisher(String, '/some directory/amazing_bool', )
        timer_period = 0.5 

        # no timer callback

def main(args=None):
    rclpy.init(args=args)
    amazing_bool_publisher = AmazingBoolPulisherNode()
    rclpy.spin(amazing_bool_publisher)
    rclpy.shutdown()