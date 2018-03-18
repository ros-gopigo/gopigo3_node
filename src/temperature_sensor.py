#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from di_sensors.temp_hum_press import TempHumPress
from sensor_msgs.msg import Temperature, RelativeHumidity, FluidPressure
from std_msgs.msg import Header


def main():
    sensor = TempHumPress()

    rospy.init_node("temperature_sensor")
    pub_temperature = rospy.Publisher("~temperature", Temperature, queue_size=10)
    pub_humidity = rospy.Publisher("~humidity", RelativeHumidity, queue_size=10)
    pub_pressure = rospy.Publisher("~pressure", FluidPressure, queue_size=10)

    rate = rospy.Rate(rospy.get_param('~hz', 2))
    while not rospy.is_shutdown():
        hdr = Header(frame_id="temperature", stamp=rospy.Time.now())

        msg_temp = Temperature(header=hdr, temperature=sensor.get_temperature_celsius())
        pub_temperature.publish(msg_temp)

        msg_hum = RelativeHumidity(header=hdr, relative_humidity=sensor.get_humidity()/100.0)
        pub_humidity.publish(msg_hum)

        msg_press = FluidPressure(header=hdr, fluid_pressure=sensor.get_pressure())
        pub_pressure.publish(msg_press)

        rate.sleep()


if __name__ == '__main__':
    main()