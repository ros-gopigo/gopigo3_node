#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import sys

try:
    import gopigo3
except IOError as e:
    print("cannot find SPI device")
    sys.exit()

import rospy
from std_msgs.msg import Int8


class Robot:
    def __init__(self):
        # GoPiGo3 setup
        self.g = gopigo3.GoPiGo3()

        # ROS setup
        rospy.init_node("gopigo3")

        # low-level access
        rospy.Subscriber("motor/pwm/left", Int8, self.on_motor_pwm_left)
        rospy.Subscriber("motor/pwm/right", Int8, self.on_motor_pwm_right)

    def start(self):
        rospy.spin()

    def on_motor_pwm_left(self, msg):
        self.g.set_motor_power(self.g.MOTOR_LEFT, msg.data)

    def on_motor_pwm_right(self, msg):
        self.g.set_motor_power(self.g.MOTOR_RIGHT, msg.data)


if __name__ == '__main__':
    r = Robot()
    r.start()