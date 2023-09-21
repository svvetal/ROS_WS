#!/usr/bin/env python3

import rospy           # Import the Python library for ROS
import random          # Import the random library

from std_msgs.msg import Int32                          # Import the Int32 message from the std_msgs package

rospy.init_node('integer_generator')                    # Initiate a Node named 'integer_generator'

pub = rospy.Publisher('integers', Int32, queue_size=10) # Create a Publisher object, that will publish on the /integers topic

rate = rospy.Rate(1)                                    # Set a publish rate of 1 Hz

while not rospy.is_shutdown():           # Create a loop that will go until someone stops the program execution
    integer = random.randint(0, 100)     # Generate a random integer between 0 and 100
    pub.publish(integer)                 # Publish the integer on the /integers topic
    rate.sleep()                         # Make sure the publish rate maintains at 1 Hz

