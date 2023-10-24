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

class PlotStrategyVectors(Node):
    def __init__(self, fig, v):
        super().__init('turtlebot3_vectors')
        self.fig_ = fig
        self.v_ = v
        # define 4 strategy vectors
        self.v_gtg = np.zeros(2, dtype='float64')
        self.v_ao = np.zeros(2, dtype='float64')
        self.v_cw = np.zeros(2, dtype='float64')
        self.v_ccw = np.zeros(2, dtype='float64')

        self.global_coord = np.zeros(2, dtype='float64')

        self.vectors_subscription = self.create_subscription(Float64MultiArray, 'strategy_vectors', self._get_vectors, 10)
        self.vectors_subscription

    def _get_vectors(self, vector_coord):
        self.v_gtg[0] = vector_coord[0]
        self.v_gtg[1] = vector_coord[1]
        self.v_ao[0] = vector_coord[2]
        self.v_ao[1] = vector_coord[3]
        self.v_cw[0] = vector_coord[4]
        self.v_cw[1] = vector_coord[5]
        self.v_ccw[0] = vector_coord[6]
        self.v_ccw[1] = vector_coord[7]

        self.update_vectors()
    
    def update_vectors(self):
        X = self.global_coord[0]
        Y = self.global_coord[1]
        vectors = [self.v_gtg, self.v_ao, self.v_cw, self.v_ccw]
        U = vectors[:, 0]
        V = vectors[:, 1]

        # update plot
        self.v_.set_UVC(U, V) 
        self.fig_.canvas.draw_idle()
        plt.pause(0.1)

def init_cartesian_graph():
    X = [0, 0, 0, 0]
    Y = [0, 0, 0, 0]
    U = [0, 0, 0, 0]
    V = [0, 0, 0, 0]

    # plot
    plt.ion()
    fig = plt.figure()

    ax = fig.add_subplot(projection='rectilinear')
    colors = ['r', 'b', 'g', 'y']
    labels = ['Go to Goal', 'Avoid Obstacle', 'Follow Wall cw', 'Follow Wall ccw']
    v = ax.quiver(X, Y, U, V, color=colors, label=labels)

    ax.legend()
    # set plot parameters
    ax.set(xlim=[-1, 1], ylim=[-1, 1], xlabel='V_x [m/s]', ylabel='V_y [m/s]')
    
    plt.draw()
    return fig, v



def main():
    rclpy.init()
    # initialize plot

    [fig, v] = init_cartesian_graph()
    turtlebot3_strategy_vectors = PlotStrategyVectors(fig, v)
    
    while rclpy.ok():
        rclpy.spin_once(turtlebot3_strategy_vectors) # Trigger callback processing and update plot
        # print('wef')
        
        

	#Clean up and shutdown.
    turtlebot3_strategy_vectors.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()