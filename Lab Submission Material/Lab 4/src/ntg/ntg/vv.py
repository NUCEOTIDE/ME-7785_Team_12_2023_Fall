#!/user/bin/env python
# import ros / turtlebot3 related libraries
import rclpy

# import node:
from rclpy.node import Node # get_object_range is a node under rclpy.node

# import messages (Twist under topic /cmd_vel, type sensor_msgs.msg)
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray

# import QoS profiles
from rclpy.qos import qos_profile_sensor_data, QoSProfile

# import plotting / math related libraries
import matplotlib.pyplot as plt
import numpy as np
import math

class PlotStrategyVectorEndPoints(Node):
    def __init__(self):
        super().__init__('turtlebot3_vectors')
        # define 4 strategy vectors
        self.v_gtg = np.zeros(2, dtype='float64')
        self.v_ao = np.zeros(2, dtype='float64')
        self.v_cw = np.zeros(2, dtype='float64')
        self.v_ccw = np.zeros(2, dtype='float64')
        self.vectors = np.zeros((4,2))


        self.global_coord = np.zeros(2, dtype='float64')

        self.vectors_subscription = self.create_subscription(Float64MultiArray, 'strategy_vectors', self._get_vectors, 10)
        self.vectors_subscription


        # publish to topic /object_range
        # self.publisher_ = self.create_publisher(LaserScan, '/object_range', 10)

    def _get_vectors(self, vector_coord1):
        vector_coord = vector_coord1.data
        self.v_gtg[0] = -vector_coord[1]
        self.v_gtg[1] = vector_coord[0]
        self.v_ao[0] = -vector_coord[3]
        self.v_ao[1] = vector_coord[2]
        self.v_cw[0] = -vector_coord[5]
        self.v_cw[1] = vector_coord[4]
        self.v_ccw[0] = -vector_coord[7]
        self.v_ccw[1] = vector_coord[6]
        self.vectors = np.array([self.v_gtg.tolist(), self.v_ao.tolist(), self.v_cw.tolist(), self.v_ccw.tolist()])
        


def init_polar_graph():
    X = [0, 0, 0, 0]
    Y = [0, 0, 0, 0]
    U = [0, 0, 0, 0]
    V = [0, 0, 0, 0]

    colors = ['r', 'b', 'g', 'y']
    labels = ['Go to Goal', 'Avoid Obstacle', 'Follow Wall cw', 'Follow Wall ccw','origin']
    
    # plot
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(projection='rectilinear')
    c = ax.scatter(U, V, c=colors, cmap='hsv', alpha=0.75)
    ax.axhline(y=0, color='k')
    ax.axvline(x=0, color='k')

    # set plot parameters
    ax.set(xlim=[-5, 5], ylim=[-5, 5], xlabel='V_y [m]', ylabel='V_x [m]')
    legend1 = ax.legend(labels,
                    loc="lower left", title="Vector Types")
    ax.add_artist(legend1)
    ax.grid(True)

    plt.draw()
    return fig, c


def update_polar_graph(c, fig, vectors):
    # update plot

    U = vectors[:, 0]
    V = vectors[:, 1]
    c.set_offsets(np.c_[U, V])
    fig.canvas.draw_idle()
    plt.pause(0.1)
    

def main():
    rclpy.init()
    # initialize plot
    fig, c = init_polar_graph()
    turtlebot3_range = PlotStrategyVectorEndPoints()
    
    while rclpy.ok():
        rclpy.spin_once(turtlebot3_range) # Trigger callback processing and update plot
        # print(turtlebot3_range.theta_)
        # print(turtlebot3_range.r_)
        update_polar_graph(c, fig, turtlebot3_range.vectors)
        # print('wef')


    # implement code for matching the camera and LiDAR here
        

	#Clean up and shutdown.
    turtlebot3_range.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
