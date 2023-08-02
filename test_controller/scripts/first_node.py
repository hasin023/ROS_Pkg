#!/usr/bin/env python3

import rospy

if __name__ == '__main__':
    rospy.init_node('First_node')
    rospy.loginfo("First node started")

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.loginfo("Goodbye World")
        rate.sleep()
