#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose


def pose_callback(data: Pose):
    rospy.loginfo("x: %f, y: %f", data.x, data.y)


if __name__ == '__main__':
    rospy.init_node('turtle_pose_subscriber')
    subscriber = rospy.Subscriber(
        "/turtle1/pose", Pose, callback=pose_callback)

    rospy.loginfo("Started pose subscriber node")
    rospy.spin()
