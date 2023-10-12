#!/user/bin/env python
import rclpy
import cv2
from cv_bridge import CvBridge

import numpy as np

# for PID control
from scipy.integrate import odeint

import rospy

# import node:
from rclpy.node import node # find_object is a node under rclpy.node

# import messages (Point under topic /object_coordinates, Twist under topic /cmd_vel)
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

# import QoS
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import sys

# define PID parameters
time = 0
integral = 0
time_prev = -1e-6
e_prev = 0

class RotateRobot(Node):

    def __init__(self):
        super().__init__('rotate_turtlebot3')
        self.subscription = self.create_subscription(Point, '/object_coordinates', self._rotate_callback, 10)
        self.subscription
        
        # create velocity publisher
        self._vel_publish = self.create_publisher(Twist, '/cmd_vel', 5)
    
    def _rotate_callback(self, coordinate_msg):
        self.get_logger().info('Object coordination on the screen: "%f"' % coordinate_msg.x)
        self.get_logger().info('Robot velocities: "%s"' % Twist)
        
        # determine the Turtlebot3 movement according to the object coordination
        if(coordinate_msg.x > 0):
            self.rotate_clockwise()
        elif(coordinate_msg.x < 0):
            self.rotate_counterclockwise()
        else:
            self.stop()
            
    def rotate_clockwise(self):
        move = Twist()
        move.linear.x=0
        move.angular.z=-0.5
        self._vel_publish.publish(move)

    def rotate_counterclockwise(self):
        move = Twist()
        move.linear.x=0
        move.angular.z=0.5
        self._vel_publish.publish(move)
    def stop(self):
        move = Twist()
        move.linear.x=0
        move.angular.z=0.0
        self._vel_publish.publish(move)

def main():
    rclpy.init() #init routine needed for ROS2.
    rotate_turtlebot3 = RotateRobot() #Create class object to be used.
    
    rclpy.spin(rotate_turtlebot3) # Trigger callback processing.
        

	#Clean up and shutdown.
    rotate_turtlebot3.destroy_node()  
    rclpy.shutdown()

if __name__ == '__main__':
    main()