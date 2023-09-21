#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
import tf
from math import sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
import time
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from tf.transformations import quaternion_from_euler

# PID Values for distance
kp_distance = 1
ki_distance = 0.01
kd_distance = 0.5

# PID Values for angle
kp_angle = 1
ki_angle = 0.03
kd_angle = 0.05

class GotoGoal():
    def __init__(self):
        rospy.init_node('goto_goal', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        position = Point()
        move_cmd = Twist()
        r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'

        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

        (position, rotation) = self.get_odom()

        last_rotation = 0

        (goal_x, goal_y, goal_z) = self.get_key()
        if goal_z > 180 or goal_z < -180:
            print("Invalid goal angle")
            self.shutdown()
        goal_z = np.deg2rad(goal_z)

        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        distance = goal_distance
        previos_distance = 0
        total_distance = 0

        previos_angle = 0
        total_angle = 0

        while distance > 0.05:
            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y

            path_angle = atan2(goal_y - y_start, goal_x - x_start)

            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2*pi + path_angle
                elif goal_y >= 0 and y_start > goal_y:
                    path_angle = 2*pi + path_angle
            if last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
            elif last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation

            diff_angle = path_angle - rotation
            diff_distance = distance - previos_distance

            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))

            # PID Equation for distance
            control_signal_distance = kp_distance*distance + ki_distance*total_distance + kd_distance*diff_distance

            # PID Equation for angle
            control_signal_angle = kp_angle*path_angle + ki_angle*total_angle + kd_angle*(diff_angle)

            move_cmd.angular.z = (control_signal_angle) - rotation

            move_cmd.linear.x = min(control_signal_distance, 0.1)

            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

            last_rotation = rotation
            self.cmd_vel.publish(move_cmd)
            r.sleep()
            previos_distance = distance
            total_distance += distance
            print("Current position and rotation are : ",(position, rotation))

        (position, rotation) = self.get_odom()
        print("Current position and rotation are : ",(position, rotation))

        print("Reached goal")

        # Logic to rotate the robot to the goal angle
        while abs(rotation - goal_z) > 0.05 :
            (position, rotation) = self.get_odom()
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    move_cmd.linear.x = 0
                    move_cmd.angular.z = 0.5
                else:
                    move_cmd.linear.x = 0
                    move_cmd.angular.z = -0.5
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    move_cmd.linear.x = 0
                    move_cmd.angular.z = -0.5
                else:
                    move_cmd.linear.x = 0
                    move_cmd.angular.z = 0.5

            self.cmd_vel.publish(move_cmd)
            r.sleep()


        self.cmd_vel.publish(Twist())
        return
    
    def get_key(self):
        global x_input, y_input, z_input
        x = x_input
        y = y_input
        z = z_input
        if x == 's':
            self.shutdown()
        x, y, z = [float(x), float(y), float(z)]
        return x, y, z
    
    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        
        return (Point(*trans), rotation[2])
    
    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


print("Enter x start position")
x_start = float(input())

print("Enter y start position")
y_start = float(input())

print("Enter start angle")
start_angle = float(input())
start_angle = np.deg2rad(start_angle)

print("Initial Pose is :")
print("(X, Y, Theta): ", x_start, y_start, start_angle)

print("Enter final x position")
x_final = float(input())
print("Enter final y position")
y_final = float(input())
print("Enter final angle")
angle_final = float(input())

final = [x_final, y_final, angle_final]
final_position = np.array(final)

x_input = final_position[0]
y_input = final_position[1]
z_input = final_position[2]

q = quaternion_from_euler(0, 0, start_angle)

state_msg = ModelState()
state_msg.model_name = 'turtlebot3_burger'
state_msg.pose.position.x = x_start
state_msg.pose.position.y = y_start
state_msg.pose.position.z = 0

state_msg.pose.orientation.x = q[0]
state_msg.pose.orientation.y = q[1]
state_msg.pose.orientation.z = q[2]
state_msg.pose.orientation.w = q[3]

set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
resp = set_state(state_msg)
print(resp)

time.sleep(5)

while not rospy.is_shutdown():
    GotoGoal()
