#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class obstacle_stop:
    def __init__(self):
        rospy.init_node('scan_node', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.linear_speed = 0.1
        self.is_obstacle = False
        self.threshold_distance = 0.2

    def scan_callback(self, msg):
        self.is_obstacle = False
        rospy.loginfo(msg.ranges[0])
        if msg.ranges[0] < self.threshold_distance:
            rospy.loginfo("Obstacle detected")
            self.is_obstacle = True
        else:
            self.is_obstacle = False

    def move_forward(self):
        rate = rospy.Rate(10)
        vel = Twist()

        while not rospy.is_shutdown():
            if self.is_obstacle:
                vel.linear.x = 0.0
                vel.angular.z = 0.0
            else:
                vel.linear.x = self.linear_speed
                vel.angular.z = 0.0
            self.pub.publish(vel)
            rate.sleep()

if __name__ == '__main__':
    try:
        obstacle_stop().move_forward()
    except rospy.ROSInterruptException:
        pass

