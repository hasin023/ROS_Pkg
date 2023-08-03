#!/usr/bin/env python3

import rospy

from math import radians
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

previous_x = 0


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

    for i in instructions:
        if i == 'F':
            # call_set_pen_service(255, 255, 255, 5, 0)
            rospy.logwarn("Moving forward")
            publisher.publish(move_cmd)
            rate.sleep()

        elif i == 'L':
            for x in range(0, 10):
                call_set_pen_service(255, 0, 0, 5, 0)
                rospy.logwarn("Turning Left")
                publisher.publish(turn_L_cmd)
                rate.sleep()

        elif i == 'R':
            for x in range(0, 10):
                call_set_pen_service(0, 255, 0, 5, 0)
                rospy.logwarn("Turning Right")
                publisher.publish(turn_R_cmd)
                rate.sleep()


def avoid_border(data):
    avoid_cmd = Twist()

    if data.x > 9.5 or data.x < 1.5 or data.y > 9.5 or data.y < 1.5:
        rospy.logwarn("Danger zone detected! Changing direction")
        avoid_cmd.linear.x = 1.0
        avoid_cmd.angular.z = - 1.3
    else:
        rospy.loginfo("Moving forward")
        avoid_cmd.linear.x = 2.0
        avoid_cmd.angular.z = 0.0

    publisher.publish(avoid_cmd)

    global previous_x
    if data.x >= 5.5 and previous_x < 5.5:
        rospy.logwarn("Crossed the middle line")
        call_set_pen_service(0, 255, 0, 5, 0)
    elif data.x <= 5.5 and previous_x > 5.5:
        rospy.logwarn("Crossed the middle line")
        call_set_pen_service(255, 0, 0, 5, 0)
    previous_x = data.x


def draw_square():

    rate = rospy.Rate(5)

    move_cmd = Twist()
    move_cmd.linear.x = 1.0

    turn_cmd = Twist()
    turn_cmd.linear.x = 0
    turn_cmd.angular.z = radians(45)

    while not rospy.is_shutdown():
        # Forward
        rospy.loginfo("Going Straight")
        for x in range(0, 10):
            publisher.publish(move_cmd)
            rate.sleep()

        # Turn 90 degrees
        rospy.loginfo("Turning")
        for x in range(0, 10):
            publisher.publish(turn_cmd)
            rate.sleep()


def pose_callback(pose: Pose):

    # avoid_border(pose)
    # draw_square()

    instructions = "FFFLFFFRFFFL"  # PLUS (+) Sign
    process_instructions(instructions)


if __name__ == '__main__':
    rospy.init_node('true_controller', anonymous=True)

    publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

    subscriber = rospy.Subscriber(
        "/turtle1/pose", Pose, callback=pose_callback)

    rospy.loginfo("Started pose subscriber node")
    rospy.spin()
