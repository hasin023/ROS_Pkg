#!/usr/bin/env python3

import rospy

from math import radians
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen


def call_set_pen_service(r, g, b, width, off):
    rospy.wait_for_service("/turtle1/set_pen")
    try:
        set_pen = rospy.ServiceProxy("/turtle1/set_pen", SetPen)
        set_pen(r, g, b, width, off)
    except rospy.ServiceException as e:
        rospy.logerr(e)


def process_instructions(instructions):

    rate = rospy.Rate(5)

    move_cmd = Twist()
    move_cmd.linear.x = 2.0

    turn_R_cmd = Twist()
    turn_R_cmd.linear.x = 0
    turn_R_cmd.angular.z = radians(-45)

    turn_L_cmd = Twist()
    turn_L_cmd.linear.x = 0
    turn_L_cmd.angular.z = radians(45)

    stop_cmd = Twist()
    stop_cmd.linear.x = 0.0
    stop_cmd.linear.y = 0.0

    for i in instructions:
        if i == 'S' or i == 's':
            rospy.logwarn("Stopping")
            publisher.publish(stop_cmd)
            break

        if i == 'F' or i == 'f':
            rospy.logwarn("Moving forward")
            publisher.publish(move_cmd)
            rate.sleep()

        elif i == 'L' or i == 'l':
            for x in range(0, 10):
                call_set_pen_service(255, 0, 0, 5, 0)
                rospy.logwarn("Turning Left")
                publisher.publish(turn_L_cmd)
                rate.sleep()

        elif i == 'R' or i == 'r':
            for x in range(0, 10):
                call_set_pen_service(0, 255, 0, 5, 0)
                rospy.logwarn("Turning Right")
                publisher.publish(turn_R_cmd)
                rate.sleep()


def pose_callback(pose: Pose):

    instructions = input("Please provide Instructions: ")
    process_instructions(instructions)
    rospy.signal_shutdown("Instructions processed")


if __name__ == '__main__':
    rospy.init_node('true_controller', anonymous=True)

    publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

    subscriber = rospy.Subscriber(
        "/turtle1/pose", Pose, callback=pose_callback)

    rospy.loginfo("Started pose subscriber node")
    rospy.spin()
