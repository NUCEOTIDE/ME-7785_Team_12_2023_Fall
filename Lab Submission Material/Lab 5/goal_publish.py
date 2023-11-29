#!/user/bin/env python
# import ros / turtlebot3 related libraries
import rclpy

# import node:
from rclpy.node import Node # get_object_range is a node under rclpy.node

# import messages (Feedback messages, nav2 messages, )
from nav2_msgs.action._navigation_to_pose import NavigateToPose_FeedbackMessage
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

# import QoS profiles

# import plotting / math related libraries
import matplotlib.pyplot as plt
import numpy as np
import math

class GoalPublish(Node):
    def __init__(self, fig, c):
        super().__init__('turtlebot3_range')
        

        # define class variables
        self.goal_list = []
        self.goal_ = PoseStamped()
        self.num_goal = 0
        self.pub_goal = 0
        self.map_name = 'map'
        self.dist_threshold = 0.25 # unit: m
        self.time_threshold = 5 # unit: s
        self.is_reached = False

        self.feedback_msg = NavigateToPose_FeedbackMessage()

        # subscribe to input topics: /clicked_point and /navigate_to_pose/_action/feedback
        self.subscription_1 = self.create_subscription(Point, '/clicked_point', self._get_goal, 10)
        self.subscription_1
        self.subscription_2 = self.create_subscription(NavigateToPose_FeedbackMessage, '/navigate_to_pose/_action/feedback', self._get_feedback, 10)
        self.subscription_2

        # publish to topic /goal_pose
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def _get_goal(self, goal):
        self.num_goal = self.num_goal + 1
        # modify class variables
        self.goal_.header.frame_id = self.map_name
        self.goal_.pose.position = goal # write position into
        self.goal_.pose.orientation.x = 0.0
        self.goal_.pose.orientation.y = 0.0
        self.goal_.pose.orientation.z = 0.0
        self.goal_.pose.orientation.w = 1.0

        self.goal_list.append(self.goal_) # append to new goal
        self.get_logger().info('New goal appended, number of goals : ' + str(self.num_goal))
        
    def _get_feedback(self, feedback_msg):
        # implement feedback related callback here.
        self.feedback_msg = feedback_msg
        self.get_logger().info('Remaining Distance: ' + str(feedback_msg.feedback.distance_remaining) + 'm')
        self.get_logger().info('Remaining Time: ' + str(feedback_msg.feedback.time_remaining) + 's')

        if self.feedback_msg.feed_back.distance_remaining <= self.dist_threshold:
            self.is_reached = True
        

def main():
    rclpy.init()

    turtlebot3_goal = GoalPublish()
    
    while rclpy.ok():
        rclpy.spin_once(turtlebot3_goal) # Trigger callback processing and update plot       
        if turtlebot3_goal.num_goal == 3:
            
            turtlebot3_goal.num_goal = -1 # enter navigate to goal state
            turtlebot3_goal.get_logger().info('3 goals detected, start publishing goals')
            
        # publish new goal if system is in navigate to goal state and current goal is reached
        if turtlebot3_goal.num_goal == -1 and turtlebot3_goal.is_reached: 

            if turtlebot3_goal.pub_goal == 2: # if the last goal is reached

                turtlebot3_goal.num_goal = 0
                turtlebot3_goal.pub_goal = 0
            else: # if current goal is not the last goal, publish the next goal
                turtlebot3_goal.publisher_.publish(turtlebot3_goal.goal_list[turtlebot3_goal.pub_goal])
                turtlebot3_goal.pub_goal = turtlebot3_goal.pub_goal + 1
                turtlebot3_goal.get_logger().info('Goal ' + str(turtlebot3_goal.pub_goal) + ' published')
            turtlebot3_goal.is_reached = False
            


	#Clean up and shutdown.
    turtlebot3_goal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
