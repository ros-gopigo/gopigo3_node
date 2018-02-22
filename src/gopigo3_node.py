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
from std_msgs.msg import Int8, Float64


class Robot:
    # short variables
    ML = gopigo3.GoPiGo3.MOTOR_LEFT
    MR = gopigo3.GoPiGo3.MOTOR_RIGHT

    def __init__(self):
        # GoPiGo3 and ROS setup
        self.g = gopigo3.GoPiGo3()
        rospy.init_node("gopigo3")

        # subscriber
        rospy.Subscriber("motor/pwm/left", Int8, self.on_motor_pwm_left)
        rospy.Subscriber("motor/pwm/right", Int8, self.on_motor_pwm_right)

        # publisher
        self.pub_enc_l = rospy.Publisher('motor/encoder/left', Float64, queue_size=10)
        self.pub_enc_r = rospy.Publisher('motor/encoder/right', Float64, queue_size=10)

        # main loop
        rate = rospy.Rate(10)   # in Hz
        while not rospy.is_shutdown():
            self.pub_enc_l.publish(Float64(data=self.g.get_motor_encoder(self.ML)))
            self.pub_enc_r.publish(Float64(data=self.g.get_motor_encoder(self.MR)))

            rate.sleep()

    def on_motor_pwm_left(self, msg):
        self.g.set_motor_power(self.ML, msg.data)

    def on_motor_pwm_right(self, msg):
        self.g.set_motor_power(self.MR, msg.data)


if __name__ == '__main__':
    r = Robot()
