#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from di_sensors.light_color_sensor import LightColorSensor
from std_msgs.msg import ColorRGBA
from std_srvs.srv import SetBool


def main():
    sensor = LightColorSensor()

    rospy.init_node("colour_sensor")
    pub_colour = rospy.Publisher("~colour", ColorRGBA, queue_size=10)

    def on_set_led(req):
        sensor.set_led(req.data)
        return [True, ""]

    srv_led = rospy.Service('~set_led', SetBool, on_set_led)

    msg_colour = ColorRGBA()

    rate = rospy.Rate(rospy.get_param('~hz', 30))
    while not rospy.is_shutdown():
        red, green, blue, alpha = sensor.get_raw_colors()
        msg_colour.r = red
        msg_colour.g = green
        msg_colour.b = blue
        msg_colour.a = alpha

        pub_colour.publish(msg_colour)

        rate.sleep()


if __name__ == '__main__':
    main()