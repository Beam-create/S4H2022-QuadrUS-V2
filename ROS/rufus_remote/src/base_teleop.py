#!/usr/bin/env python
"""
File to convert joy messages to direction(twist) messages
Subscriber : joy
Publisher : rufus/base_teleop
"""

from sympy import true
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class base_teleop:
    
    def __init__(self):
        """
        Initialize subscriber, pulisher and node
        """
        self.joy_sub = rospy.Subscriber("joy", Joy, self.cb)
        self.twist_pub = rospy.Publisher('rufus/base_teleop', Twist, queue_size=200)

    def cb(self,data):
        """
        Convert joy messages to relevent twist message
        """
        
        twist = Twist()
        twist.linear.x = data.axes[1]
        twist.linear.y = -1*data.axes[0]
        twist.angular.z = -1*data.axes[3]
        self.twist_pub.publish(twist)

if __name__=='__main__':
    rospy.init_node('base_teleop', anonymous=true)
    bt = base_teleop()
    rospy.spin()