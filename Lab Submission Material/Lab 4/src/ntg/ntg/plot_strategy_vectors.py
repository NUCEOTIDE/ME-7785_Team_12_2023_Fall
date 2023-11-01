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
    def __init__(self, fig, v, c):
        super().__init__('turtlebot3_vectors')
        self.fig_ = fig
        self.v_ = v
        self.c_ = c
        # define 4 strategy vectors
        self.v_gtg = np.zeros(2, dtype='float64')
        self.v_ao = np.zeros(2, dtype='float64')
        self.v_cw = np.zeros(2, dtype='float64')
        self.v_ccw = np.zeros(2, dtype='float64')
        self.vectors = np.zeros((4,2))
        self.current_state = -1

        self.global_coord = np.zeros(2, dtype='float64')

        self.vectors_subscription = self.create_subscription(Float64MultiArray, 'strategy_vectors', self._get_vectors, 10)
        self.vectors_subscription

        self.current_state_subscription = self.create_subscription(Point, 'vector', self._get_state, 10)
        self.current_state_subscription

    def _get_vectors(self, vector_coord1):
        vector_coord = vector_coord1.data
        self.v_gtg[0] = vector_coord[1]
        self.v_gtg[1] = -vector_coord[0]
        self.v_ao[0] = vector_coord[3]
        self.v_ao[1] = -vector_coord[2]
        self.v_cw[0] = vector_coord[5]
        self.v_cw[1] = -vector_coord[4]
        self.v_ccw[0] = vector_coord[7]
        self.v_ccw[1] = -vector_coord[6]
        self.vectors = np.array([self.v_gtg.tolist(), self.v_ao.tolist(), self.v_cw.tolist(), self.v_ccw.tolist()])
        self.update_vectors()
    
    def _get_state(self, current_state):
        self.current_state = current_state.z

    def update_vectors(self):
        X = self.global_coord[0]
        Y = self.global_coord[1]
        U = self.vectors[:, 0]
        V = self.vectors[:, 1]

        

        # update plot
        self.v_.set_UVC(U, V) # update vectors
        if(self.current_state != -1):
            self.c_.set_offsets(np.c_[U[self.current_state], V[self.current_state]]) # update current state
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
    labels = ['y-axis', 'x-axis', 'Strategy Vectors', 'Current State Vector']
    vector_labels = ['Go to Goal', 'Avoid Obstacle', 'Follow Wall cw', 'Follow Wall ccw']
    v = ax.quiver(X, Y, U, V, color=colors, angles='xy', scale_units='xy', scale=1, width=0.003, label=labels)
    
    c = ax.scatter(U[1], V[1], c='r', cmap='hsv', s=3, alpha=0.75, edgecolors='k')
    plt.quiverkey(v, .1, .9, .2, vector_labels[0], color='r', labelpos='E')
    plt.quiverkey(v, .1, .83, .2, vector_labels[1], color='b', labelpos='E')
    plt.quiverkey(v, .1, .76, .2, vector_labels[2], color='g', labelpos='E')
    plt.quiverkey(v, .1, .69, .2, vector_labels[3], color='y', labelpos='E')
    
    ax.axhline(y=0, linewidth=0.01, color='k')
    ax.axvline(x=0, linewidth=0.01, color='k')
    legend1 = ax.legend(labels,
                    loc="lower left", title="Legend Types")
    # set plot parameters
    ax.set(xlim=[-3, 3], ylim=[-3, 3], xlabel='S_x [m]', ylabel='S_y [m]')
    ax.add_artist(legend1)
    ax.grid(True)
    plt.draw()
    return fig, v, c



def main():
    rclpy.init()
    # initialize plot

    [fig, v, c] = init_cartesian_graph()
    turtlebot3_strategy_vectors = PlotStrategyVectors(fig, v, c)
    
    while rclpy.ok():
        rclpy.spin_once(turtlebot3_strategy_vectors) # Trigger callback processing and update plot
        # print('wef')
        
        

	#Clean up and shutdown.
    turtlebot3_strategy_vectors.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
