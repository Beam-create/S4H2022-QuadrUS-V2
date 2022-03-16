#!/usr/bin/env python
"""
File to convert joy messages to joint angles messages
Subscriber : joy
Publisher : rufus/bras_teleop
"""

from sympy import false, true
import rospy
from sensor_msgs.msg import Joy
from rufus_master.msg import bras_commands
from rufus_master.msg import Feedback_arduino_msgs

class bras_teleop:

    def __init__(self):
        """
        Initialize subscriber, pulisher and node
        """
        self.joy_sub = rospy.Subscriber("joy", Joy, self.cb_joy)
        self.prevAng_sub = rospy.Subscriber("rufus/arduino_feedback", Feedback_arduino_msgs, self.cb_feed)
        self.ang_pub = rospy.Publisher("rufus/bras_teleop", bras_commands, queue_size=50)
        #self.angles = bras_commands()
        self.commands = bras_commands()
        self.ang_inc = 5.0

    def cb_feed(self, data):
        """
        Convert joy messages to relevent twist message
        """
        #Fetch the current state of robot arm from Arduino feedback
        self.commands.q1 = data.q1
        self.commands.q2 = data.q2
        self.commands.q3 = data.q3
        self.commands.effector = data.effector
        
    def cb_joy(self, data):
        isTriggered = false

        # Tester config manette pour attribuer les valeurs a angles.q*
        #q1
        if(data.buttons[15]):
            self.commands.q1 = self.commands.q1 + self.ang_inc
            isTriggered = true
        if(data.buttons[16]):
            self.commands.q1 = self.commands.q1 - self.ang_inc
            isTriggered = true

        #q2
        if(data.buttons[13]):
            self.commands.q2 = self.commands.q2 + self.ang_inc
            isTriggered = true
        if(data.buttons[14]):
            self.commands.q2 = self.commands.q2 - self.ang_inc
            isTriggered = true

        #q3
        if(data.buttons[2]):
            self.commands.q3 = self.commands.q3 + self.ang_inc
            isTriggered = true
        if(data.buttons[0]):
            self.commands.q3 = self.commands.q3 - self.ang_inc
            isTriggered = true

        #effector
        if(data.buttons[5]):
            self.commands.effector = true
            isTriggered = true
        if(data.buttons[4]):
            self.commands.effector = false
            isTriggered = true
        if isTriggered:
            self.ang_pub.publish(self.commands)


if __name__=='__main__':
    try:
        bras_t = bras_teleop()
        rospy.init_node('bras_teleop', anonymous=true)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

