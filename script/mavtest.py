#!/usr/bin/env python
# vim:set ts=4 sw=4 et:

import sys
import argparse
import rospy
import mavros

from std_msgs.msg import Header, Float64
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Quaternion, Point
from mavros_msgs.msg import OverrideRCIn
from mavros import command
from mavros import setpoint as SP

def main():
    override_pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)
    rospy.init_node("mavtest")
    mavros.set_namespace("/mavros")

    rc = OverrideRCIn()

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rc.channels = [1000,1000,1000,1000,1000,1000,1000,1000]
        rospy.loginfo(rc)
        override_pub.publish(rc)
        rate.sleep()

    #rospy.spin()

if __name__ == '__main__':
    main()

