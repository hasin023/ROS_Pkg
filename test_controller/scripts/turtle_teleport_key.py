#!/usr/bin/env python3


from __future__ import print_function
from pynput.keyboard import Key, Listener

import rospy
import sys
import select
import os
if os.name == 'nt':
    import msvcrt
else:
    import tty
    import termios
from geometry_msgs.msg import Twist

BURGER_MAX_LIN_VEL = 2.0
BURGER_MAX_ANG_VEL = 2.8

WAFFLE_MAX_LIN_VEL = 2.0
WAFFLE_MAX_ANG_VEL = 1.8

LIN_VEL_STEP_SIZE = 0.2
ANG_VEL_STEP_SIZE = 0.1

msg = """
Moving around:
        w
   a    s    d
   
---------------------------
stop when key released
ESC to quit
"""

e = """
Communications Failed
"""


class Teleop:
    def __init__(self):
        print(msg)
        self.keys = set()
        self.status = 0
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0

        with Listener(
            on_press=self.on_press,
            on_release=self.on_release
        ) as listener:
            listener.join()

    def vels(self, target_linear_vel, target_angular_vel):
        return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel, target_angular_vel)

    def constrain(self, input, low, high):
        if input < low:
            input = low
        elif input > high:
            input = high
        else:
            input = input

        return input

    def checkLinearLimitVelocity(self, vel):
        if turtlebot3_model in ["waffle", "waffle_pi"]:
            max_lin_vel = WAFFLE_MAX_LIN_VEL
        else:
            max_lin_vel = BURGER_MAX_LIN_VEL

        vel = self.constrain(vel, -max_lin_vel, max_lin_vel)
        return vel

    def checkAngularLimitVelocity(self, vel):
        if turtlebot3_model in ["waffle", "waffle_pi"]:
            max_ang_vel = WAFFLE_MAX_ANG_VEL
        else:
            max_ang_vel = BURGER_MAX_ANG_VEL

        vel = self.constrain(vel, -max_ang_vel, max_ang_vel)
        return vel

    def move(self):
        global pub
        try:
            keys = ''.join(self.keys)
            if keys.find('w') == -1 and keys.find('s') == -1:
                self.target_linear_vel = 0.0
            else:
                sign = 1 if keys.find('w') > keys.find('s') else -1
                self.target_linear_vel = self.checkLinearLimitVelocity(
                    self.target_linear_vel + sign * LIN_VEL_STEP_SIZE)

            if keys.find('a') == -1 and keys.find('d') == -1:
                self.target_angular_vel = 0.0
            else:
                sign = 1 if keys.find('a') > keys.find('d') else -1
                self.target_angular_vel = self.checkAngularLimitVelocity(
                    self.target_angular_vel + sign * ANG_VEL_STEP_SIZE)

            self.status += 1
            print(self.vels(self.target_linear_vel, self.target_angular_vel))

            if self.status == 100:
                print(msg)
                self.status = 0

            twist = Twist()
            twist.linear.x = self.target_linear_vel
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.z = self.target_angular_vel
            pub.publish(twist)

        except Exception as err:
            print(e)
            print(err)

    def on_press(self, key):
        # to remove input buffer
        if os.name == 'nt':
            msvcrt.getch()
        else:
            tty.setraw(sys.stdin.fileno())
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        if key == Key.esc:
            return False

        if not hasattr(key, 'char'):
            return True

        key = key.char
        if key not in 'wasd':
            return True

        if key == 'w' and 's' in self.keys:
            self.keys.remove('s')
        if key == 'a' and 'd' in self.keys:
            self.keys.remove('d')
        if key == 's' and 'w' in self.keys:
            self.keys.remove('w')
        if key == 'd' and 'a' in self.keys:
            self.keys.remove('a')

        self.keys.add(key)
        self.move()

    def on_release(self, key):
        if not hasattr(key, 'char'):
            return True

        key = key.char
        if key in self.keys:
            self.keys.remove(key)
            self.move()


if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turtle_teleop')
    turtlebot3_model = rospy.get_param("model", "burger")
    topic_type = rospy.get_param("topic_type", "turtlebot")

    topic_name = '/turtle1/cmd_vel'
    pub = rospy.Publisher(topic_name, Twist, queue_size=10)

    Teleop()

    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
