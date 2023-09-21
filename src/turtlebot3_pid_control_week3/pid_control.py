#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from math import pow, atan2, sqrt

class PIDControl():

    def __init__(self):
        # Creating our node,publisher and subscriber
        rospy.init_node('pid_control', anonymous=True)

        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


        # A subscriber to the topic '/gazebo/model_states'. self.update_pose is called
        # when a message of type ModelStates is received.
        self.pose_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.update_pose)

        self.current_state = ModelStates()
        self.rate = rospy.Rate(10)

    def update_pose(self,data):
        """Callback function which is called when a new message of type ModelStates is
        received by the subscriber."""
        self.current_state = data

        # Round position values to 4 decimal places
        self.current_state.pose[1].position.x = round(self.current_state.pose[1].position.x, 4)
        self.current_state.pose[1].position.y = round(self.current_state.pose[1].position.y, 4)

    def euclidean_distance(self, goal_state):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_state.pose[1].position.x - self.current_state.pose[1].position.x), 2) +
                    pow((goal_state.pose[1].position.y - self.current_state.pose[1].position.y), 2))
    
    def linear_vel(self, goal_state, constant=1.5):
        return constant * self.euclidean_distance(goal_state)
    
    def steering_angle(self, goal_state):
        return atan2(goal_state.pose[1].position.y - self.current_state.pose[1].position.y, goal_state.pose[1].position.x - self.current_state.pose[1].position.x)
    
    def angular_vel(self, goal_state, constant=6):
        return constant * (self.steering_angle(goal_state) - self.current_state.pose[1].orientation.z)
    
    def move_to_goal(self):
        goal_state = ModelStates()

        # Get the input from the user.
        goal_state.pose[1].position.x = float(input("Set your x goal: "))
        goal_state.pose[1].position.y = float(input("Set your y goal: "))

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = input("Set your tolerance: ")

        vel_msg = Twist()

        while self.euclidean_distance(goal_state) >= distance_tolerance:

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_state)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_state)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        rospy.spin()

if __name__ == '__main__':
    try:
        x = PIDControl()
        x.move_to_goal()
    except rospy.ROSInterruptException:
        pass

