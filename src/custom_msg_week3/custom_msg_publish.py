

import rospy
from custom_msg_week3.msg import custom

def custom_msg_publish():
    pub = rospy.Publisher('custom_msg_topic', custom, queue_size=10)
    rospy.init_node('custom_msg_publish', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    cust_msg = custom()

    while not rospy.is_shutdown():
        cust_msg.user_input.data = str(input("Enter a Character: "))
        # rospy.loginfo(cust_msg)
        pub.publish(cust_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        custom_msg_publish()
    except rospy.ROSInterruptException:
        pass