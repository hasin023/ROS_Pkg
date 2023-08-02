#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


def pose_callback(pose: Pose):
    cmd = Twist()

    if pose.x > 10.0 or pose.x < 1.0 or pose.y > 10.0 or pose.y < 1.0:
        cmd.linear.x = 1.0
        cmd.angular.z = 1.3
    else:
        cmd.linear.x = 2.0
        cmd.angular.z = 0.0

    publisher.publish(cmd)


if __name__ == '__main__':
    rospy.init_node('true_controller', anonymous=True)

    publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

    subscriber = rospy.Subscriber(
        "/turtle1/pose", Pose, callback=pose_callback)

    rospy.loginfo("Started pose subscriber node")
    rospy.spin()
