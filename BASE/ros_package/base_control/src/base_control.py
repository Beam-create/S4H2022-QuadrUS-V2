#!/usr/bin/env python
"""
Node to convert direction(twist) messages to actual movement
Subscriber : rufus/base_teleop
Publisher : none
"""

from sympy import true
import rospy
from geometry_msgs.msg import Twist
import rosserial_arduino

class base_control:

    def __init__(self):
        self.twist_sub = rospy.Subscriber("rufus/base_teleop", Twist, self.cb)

    def cb(self,data):
        pass

if __name__=='__main__':
    rospy.init_node('base_control', anonymous=true)
    b_c = base_control()
    rospy.spin()