#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def input_callback(data):
    n = data.data
    rospy.loginfo("Sent: %d", n)
    transmit.publish(n)

if __name__ == '__main__':
    rospy.init_node('autopose', anonymous=True)
    rospy.Subscriber('input', Int32, input_callback)
    transmit = rospy.Publisher('transmission', Int32, queue_size=10)
    rospy.spin()

