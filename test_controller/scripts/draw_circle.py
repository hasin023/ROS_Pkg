#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.init_node('draw_circle')
    rospy.loginfo("Started drawing circle")

    publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        message = Twist()
        message.linear.x = 2.0
        message.angular.z = 1.0
        publisher.publish(message)
        rate.sleep()
