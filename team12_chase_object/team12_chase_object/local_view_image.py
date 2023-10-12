import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

import cv2
from cv_bridge import CvBridge

class Local_View_Image(Node):
    def __init__(self):
        super().__init__('local_view_image')
        self._image_subscriber = self.create_subscription(CompressedImage,'image_show',self.image_callback,10)

    def image_callback(self,CompressedImage):
        self.rgb_image = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
        cv2.imshow('real-time image',self.rgb_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    local_view_image_node = Local_View_Image()
    rclpy.spin(local_view_image_node)
    local_view_image_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


