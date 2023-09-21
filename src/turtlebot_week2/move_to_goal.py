#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist


def move_to_goal():
    rospy.init_node("move_to_goal")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    vel = Twist()
    vel.linear.x = 0.5

    while not rospy.is_shutdown():
        t0 = rospy.Time.now().to_sec()
        current_distance = 0
        linear_speed = 0.5
        goal = 2

        while current_distance < goal:
            pub.publish(vel)
            t1 = rospy.Time.now().to_sec()
            current_distance = linear_speed * (t1 - t0)

        vel.linear.x = 0
        pub.publish(vel)


if __name__ == "__main__":
    try:
        move_to_goal()
    except rospy.ROSInterruptException:
        pass
