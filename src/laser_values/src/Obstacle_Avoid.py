#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('scan_node', anonymous=True)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.max_linear_speed = 0.2  # Maximum linear speed (m/s)
        self.max_angular_speed = 1.0  # Maximum angular speed (rad/s)
        self.min_distance = 0.5  # Minimum distance to an obstacle (m)
        self.turn_duration = 1.0  # Duration for the turn (s)
        self.is_obstacle_detected = False

    def scan_callback(self, scan_msg):
        ranges = scan_msg.ranges
        front_angle_indices = range(-10, 11)  # Indices corresponding to the frontal 20 degrees of the scan
        front_ranges = [ranges[i] for i in front_angle_indices]

        if min(front_ranges) < self.min_distance:
            rospy.loginfo("Obstacle detected!")
            self.is_obstacle_detected = True
        else:
            self.is_obstacle_detected = False

    def move_forward(self):
        rate = rospy.Rate(10)  # 10 Hz loop rate
        twist_msg = Twist()

        while not rospy.is_shutdown():
            if self.is_obstacle_detected:
                self.avoid_obstacle(twist_msg)
            else:
                twist_msg.linear.x = self.max_linear_speed
                twist_msg.angular.z = 0.0

            self.vel_pub.publish(twist_msg)
            rate.sleep()

    def avoid_obstacle(self, twist_msg):
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = self.max_angular_speed

        # Turn for the specified duration
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(self.turn_duration):
            self.vel_pub.publish(twist_msg)
            rospy.sleep(0.1)

        twist_msg.angular.z = 0.0
        self.is_obstacle_detected = False

if __name__ == '__main__':
    try:
        obstacle_avoidance = ObstacleAvoidance()
        obstacle_avoidance.move_forward()
    except rospy.ROSInterruptException:
        pass

