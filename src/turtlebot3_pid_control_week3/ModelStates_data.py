#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from math import pow, atan2, sqrt

def callback(msg):
    print(msg.pose[1].position.x)

rospy.init_node('pid_control', anonymous=True)
sub = rospy.Subscriber('/gazebo/model_states', ModelStates, callback)
rospy.spin()
