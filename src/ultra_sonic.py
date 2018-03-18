#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import gopigo3
from sensor_msgs.msg import Range


def main():
    GPG = gopigo3.GoPiGo3()

    rospy.init_node("ultra_sonic")
    port_id = rospy.get_param("~port", 1)
    if port_id==1:
        port = GPG.GROVE_1
    elif port_id==2:
        port = GPG.GROVE_2
    else:
        rospy.logerr("unknown port %i", port_id)
        return

    GPG.set_grove_type(port, GPG.GROVE_TYPE.US)

    pub_distance = rospy.Publisher("~distance", Range, queue_size=10)

    msg_range = Range()
    msg_range.header.frame_id = "ultra_sonic"
    msg_range.radiation_type = Range.ULTRASOUND
    msg_range.min_range = 0.02
    msg_range.max_range = 4.3

    rate = rospy.Rate(rospy.get_param('~hz', 30))
    while not rospy.is_shutdown():
        # read distance in meter
        msg_range.range = GPG.get_grove_value(port) / 1000.0
        msg_range.header.stamp = rospy.Time.now()

        pub_distance.publish(msg_range)

        rate.sleep()


if __name__ == '__main__':
    main()