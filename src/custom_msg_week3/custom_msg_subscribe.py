#!/usr/bin/env python3

import rospy
from custom_msg_week3.msg import custom
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from math import atan2, sqrt, pow

class P_Controlled_Bot:
    def __init__(self):
        rospy.init_node('custom_msg_subscribe', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub1 = rospy.Subscriber('custom_msg_topic', custom, self.callback)
        self.sub2 = rospy.Subscriber('/gazebo/model_states', ModelStates, self.update_pose)

        self.current_pose = ModelStates()
        self.rate = rospy.Rate(10) # 10hz


    def callback(self, data):
        self.user_input = data.user_input.data
        # rospy.loginfo(self.user_input)

    def update_pose(self, data):
        self.current_pose = data

        self.current_pose.pose[1].position.x = round(self.current_pose.pose[1].position.x, 4)
        self.current_pose.pose[1].position.y = round(self.current_pose.pose[1].position.y, 4)

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.pose[1].position.x - self.current_pose.pose[1].position.x), 2) +
                    pow((goal_pose.pose[1].position.y - self.current_pose.pose[1].position.y), 2))
    
    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)
    
    def steering_angle(self, goal_pose):
        return atan2(goal_pose.pose[1].position.y - self.current_pose.pose[1].position.y, goal_pose.pose[1].position.x - self.current_pose.pose[1].position.x)
    
    def angular_vel(self, goal_pose, constant=6):
        return constant * (self.steering_angle(goal_pose) - self.current_pose.pose[1].orientation.z)
    
    def move2goal(self):
        goal_pose = ModelStates()
        goal_pose.pose[1].position.x = float(input("Set your x goal: "))
        goal_pose.pose[1].position.y = float(input("Set your y goal: "))
        distance_tolerance = float(input("Set your tolerance: "))
        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            self.pub.publish(vel_msg)
            self.rate.sleep()
        
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.pub.publish(vel_msg)

        rospy.loginfo("Goal reached")

    def Robot_Action(self,user_input):
        vel_msg = Twist()
        if user_input == ("w" or "W"):
            vel_msg.linear.x = 1
            vel_msg.angular.z = 0
        elif user_input == ("a" or "A"):
            vel_msg.linear.x = 0
            vel_msg.angular.z = 1
        elif user_input == ("s" or "S"):
            vel_msg.linear.x = -1
            vel_msg.angular.z = 0
        elif user_input == ("d" or "D"):
            vel_msg.linear.x = 0
            vel_msg.angular.z = -1
        else:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
        self.pub.publish(vel_msg)

    

if __name__ == '__main__':
    try:
        P_Controlled_Bot()
    except rospy.ROSInterruptException:
        pass