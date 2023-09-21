#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_forward():
    rospy.init_node('move_with_constant_velocity')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    vel = Twist()
    vel.linear.x = 0.5
    while not rospy.is_shutdown():
        pub.publish(vel)
        rate.sleep()


if __name__ == '__main__':
    try:
        move_forward()
    except rospy.ROSInterruptException:
        pass