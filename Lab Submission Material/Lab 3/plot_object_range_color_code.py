#!/user/bin/env python
# import ros / turtlebot3 related libraries
import rclpy

# import node:
from rclpy.node import Node # get_object_range is a node under rclpy.node

# import messages (Twist under topic /cmd_vel, type sensor_msgs.msg)
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray

# import QoS profiles
from rclpy.qos import qos_profile_sensor_data, QoSProfile

# import plotting / math related libraries
import matplotlib.pyplot as plt
import numpy as np
import math

class PlotObjectRange(Node):
    def __init__(self):
        super().__init__('turtlebot3_range')
        

        # define class variables
        self.theta_ = []
        self.r_ = []

        self.obj_boundary_left = 0
        self.obj_boundary_right = 0

        # self.obj_coord_ = Point()

        # subscribe to input topics: /scan and /object_coordinates
        self.subscription_1 = self.create_subscription(LaserScan, '/scan', self._get_scan, qos_profile=qos_profile_sensor_data)
        self.subscription_1
        
        self.subscription_2 = self.create_subscription(Float64MultiArray, 'boundary', self._set_boundary, 10)
        self.subscription_2

        # publish to topic /object_range
        # self.publisher_ = self.create_publisher(LaserScan, '/object_range', 10)

    def _get_object_coordinates(self, coordinate_msg):
        self.obj_coord_ = coordinate_msg
        # implement obj coord related callback here.


    def _get_scan(self, scan):
        # self.get_logger().info('')
        scan_filter = []
        samples = len(scan.ranges)  # The number of samples is defined in 
                                    # turtlebot3_<model>.gazebo.xacro file,
                                    # the default is 360.
        samples_view = 1            # 1 <= samples_view <= samples

        # Compute areas and colors
        theta = np.linspace(scan.angle_min, scan.angle_max, samples)
        r = scan.ranges

        
        for i in range(len(r)):
            if r[i] == float('Inf'):
                r[i] = 3.5
            elif math.isnan(r[i]):
                r[i] = 0

        # modify class variables
        self.theta_ = theta
        self.r_ = r

        
    
    # def get_object_range(self):
    #     self.publisher_.pulish() # need to figure out msg type
    
    def _set_boundary(self, boundary):
        self.obj_boundary_left = abs(int(boundary.data[0]))
        self.obj_boundary_right = abs(int(boundary.data[1]))


def init_polar_graph(theta, r, theta_max, theta_min, left, right):
    obj_left = np.zeros(right)
    obj_right = np.zeros(left)
    background = np.ones(360 - (left + right))

    colors = np.concatenate(obj_left, background, obj_right, 1)
    
    # plot
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(projection='polar')
    c = ax.scatter(theta, r, c=colors, cmap='hsv', alpha=0.75)

    # set plot parameters
    ax.set_thetamin(theta_min)
    ax.set_thetamax(theta_max)
    ax.set_theta_zero_location('N', offset=0)

    ax.set_rlim(0, 2.5)

    plt.draw()
    return fig, c


def update_polar_graph(c, fig, new_theta, new_r):
    # update plot
    c.set_offsets(np.c_[new_theta, new_r])
    fig.canvas.draw_idle()
    plt.pause(0.1)
    

def main():
    rclpy.init()
    # initialize plot
    fig, c = init_polar_graph([], [], 360, 0)
    turtlebot3_range = PlotObjectRange()
    
    while rclpy.ok():
        rclpy.spin_once(turtlebot3_range) # Trigger callback processing and update plot
        print(turtlebot3_range.theta_)
        print(turtlebot3_range.r_)
        update_polar_graph(c, fig, turtlebot3_range.theta_, turtlebot3_range.r_, 
                           turtlebot3_range.obj_boundary_left, turtlebot3_range.obj_boundary_right)
        # print('wef')


    # implement code for matching the camera and LiDAR here
        

	#Clean up and shutdown.
    turtlebot3_range.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
