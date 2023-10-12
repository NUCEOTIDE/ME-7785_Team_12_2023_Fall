#!/user/bin/env python
# import ros / turtlebot3 related libraries
import rclpy

# import node:
from rclpy.node import Node # get_object_range is a node under rclpy.node

# import messages (Twist under topic /cmd_vel, type sensor_msgs.msg)
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

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

        # self.obj_angle_range = [][]

        # self.obj_coord_ = Point()

        # subscribe to input topics: /scan and /object_coordinates
        self.subscription_1 = self.create_subscription(LaserScan, '/scan', lambda msg: self._get_scan(msg, fig, c), 10)
        self.subscription_1
        # self.subscription_2 = self.create_subscription(Point, '/object_coordinates', self._get_object_coordinates, 10)
         #self.subscription_2

        # publish to topic /object_range
        # self.publisher_ = self.create_publisher(LaserScan, '/object_range', 10)

    def _get_object_coordinates(self, coordinate_msg):
        self.obj_coord_ = coordinate_msg
        # implement obj coord related callback here.


    def _get_scan(self, scan, fig, c):
        # self.get_logger().info('')
        scan_filter = []
        samples = len(scan.ranges)  # The number of samples is defined in 
                                    # turtlebot3_<model>.gazebo.xacro file,
                                    # the default is 360.
        samples_view = 1            # 1 <= samples_view <= samples

        # Compute areas and colors
        theta = np.linspace(scan.angle_min, scan.angle_max, samples)
        r = scan.ranges

        
        for i in range(r):
            if r[i] == float('Inf'):
                r[i] = 3.5
            elif math.isnan(r[i]):
                r[i] = 0

        # modify class variables
        self.theta_ = theta
        self.r_ = r

        # update plot
        c.set_offsets(np.c_[theta, r])
        fig.canvas.draw_idle()
        plt.pause(0.1)
    
    # def get_object_range(self):
    #     self.publisher_.pulish() # need to figure out msg type
    
def init_polar_graph(theta, r, theta_max, theta_min):
    # plot
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(projection='polar')
    c = ax.scatter(theta, r, cmap='hsv', alpha=0.75)

    # set plot parameters
    ax.set_thetamin(theta_min)
    ax.set_thetamax(theta_max)
    ax.set_theta_zero_location('N', offset=0)

    plt.draw()
    return fig, c


# def update_polar_graph(c, fig, new_theta, new_r):
    

def main():
    rclpy.init()
    # initialize plot
    fig, c = init_polar_graph([], [], 360, 0)
    turtlebot3_range = PlotObjectRange(fig, c)
    
    
    rclpy.spin(turtlebot3_range) # Trigger callback processing and update plot
    print(turtlebot3_range.theta_)
    print(turtlebot3_range.r_)
    # update_polar_graph(c, fig, turtlebot3_range.theta_, turtlebot3_range.r_)
    print('wef')

    # implement code for matching the camera and LiDAR here
        

	#Clean up and shutdown.
    turtlebot3_range.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
