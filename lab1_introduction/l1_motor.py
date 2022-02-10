#! /usr/bin/env python3

import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
  
# function to create node and publish to command velocity topic
def publisher_node():

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rate =rospy.Rate(2)     # 1 Hz

    
    twist = Twist()
    twist.linear.x = 0.25
    start = time.time()
    

    while time.time() - start < 4.5:
        rospy.loginfo('moving straight...')
        pub.publish(twist)
        rate.sleep()

    twist.linear.x = 0
    twist.angular.z = 1

    start2 = time.time()
    while time.time() - start2 < 6.5:
        rospy.loginfo('turning 360 degrees...')
        pub.publish(twist)
        rate.sleep()

    rospy.loginfo('stopping...')
    twist.angular.z = 0
    pub.publish(twist)

def main():
    try:
        rospy.init_node('motor')
        publisher_node()
    except rospy.ROSInterruptException:
        pass
    

if __name__ == '__main__':
    main()