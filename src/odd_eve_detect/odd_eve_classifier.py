#!/usr/bin/env python3

import rospy                      # Import the Python library for ROS

from std_msgs.msg import Int32    # Import the Int32 message from the std_msgs package
from std_msgs.msg import String   # Import the String message from the std_msgs package

def callback(msg):                # Define a function called 'callback' that receives a parameter named 'msg'
    if msg.data % 2 == 0:         # If the 'data' field of the 'msg' parameter is even
        pub.publish(str(msg.data) + " received, is an even number")    # Publish a new message on the 'oddeven' topic
    else:
        pub.publish(str(msg.data) + " received, is a odd number")      # Publish a new message on the 'oddeven' topic

rospy.init_node('odd_eve_classifier')                        # Initiate a Node named 'odd_eve_classifier'

pub = rospy.Publisher('oddeven', String, queue_size=10)      # Create a Publisher object, that will publish on the /oddeven topic
sub = rospy.Subscriber('integers', Int32, callback)          # Create a Subscriber object that will subscribe to the /integers topic and will call the 'callback' function each time it reads something from the topic

rospy.spin()                                                 # Create a loop that will keep the program in execution until someone stops it