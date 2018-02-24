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
from std_msgs.msg import Int8, Int16, Float64


class Robot:
    # short variables
    ML = gopigo3.GoPiGo3.MOTOR_LEFT
    MR = gopigo3.GoPiGo3.MOTOR_RIGHT
    S1 = gopigo3.GoPiGo3.SERVO_1
    S2 = gopigo3.GoPiGo3.SERVO_2

    def __init__(self):
        # GoPiGo3 and ROS setup
        self.g = gopigo3.GoPiGo3()
        print("GoPiGo3 info:")
        print("Manufacturer    : ", self.g.get_manufacturer())
        print("Board           : ", self.g.get_board())
        print("Serial Number   : ", self.g.get_id())
        print("Hardware version: ", self.g.get_version_hardware())
        print("Firmware version: ", self.g.get_version_firmware())

        rospy.init_node("gopigo3")

        # subscriber
        rospy.Subscriber("motor/pwm/left", Int8, lambda msg: self.g.set_motor_power(self.ML, msg.data))
        rospy.Subscriber("motor/pwm/right", Int8, lambda msg: self.g.set_motor_power(self.MR, msg.data))
        rospy.Subscriber("motor/position/left", Int16, lambda msg: self.g.set_motor_position(self.ML, msg.data))
        rospy.Subscriber("motor/position/right", Int16, lambda msg: self.g.set_motor_position(self.MR, msg.data))
        rospy.Subscriber("servo/1", Float64, lambda msg: self.g.set_servo(self.S1, msg.data*16666))
        rospy.Subscriber("servo/2", Float64, lambda msg: self.g.set_servo(self.S2, msg.data*16666))

        # publisher
        self.pub_enc_l = rospy.Publisher('motor/encoder/left', Float64, queue_size=10)
        self.pub_enc_r = rospy.Publisher('motor/encoder/right', Float64, queue_size=10)
        self.pub_battery = rospy.Publisher('battery_voltage', Float64, queue_size=10)

        # main loop
        rate = rospy.Rate(10)   # in Hz
        while not rospy.is_shutdown():
            self.pub_enc_l.publish(Float64(data=self.g.get_motor_encoder(self.ML)))
            self.pub_enc_r.publish(Float64(data=self.g.get_motor_encoder(self.MR)))
            self.pub_battery.publish(Float64(data=self.g.get_voltage_battery()))

            rate.sleep()

        self.g.reset_all()


if __name__ == '__main__':
    r = Robot()
