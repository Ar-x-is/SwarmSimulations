#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Int32

def callback(data):
    rospy.loginfo("Received: %d", data.data)
    time.sleep(5)  # Delay for 5 seconds

    # Calculate Fibonacci
    n = data.data
    fib_result = fibonacci(n)

    # Publish Fibonacci result to "output" topic
    rospy.loginfo("Result: %d", fib_result)

def fibonacci(n):
    if n <= 0:
        return 0
    elif n == 1:
        return 1
    else:
        return fibonacci(n - 1) + fibonacci(n - 2)

if __name__ == '__main__':
    rospy.init_node('navigator', anonymous=True)
    rospy.Subscriber('transmission', Int32, callback)
    rospy.spin()

