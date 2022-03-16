#!/usr/bin/env python
"""
File to convert joy messages to joint angles messages
Subscriber : joy
Publisher : rufus/bras_teleop
"""

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
        self.ang_pub = rospy.Publisher("rufus/bras_arduino", bras_commands, queue_size=50)

        #objet message de commande
        self.commands = bras_commands()

        # Initial values of angles on start-up
        self.commands.q1 = 0.0
        self.commands.q2 = 90.0
        self.commands.q3 = 0.0
        self.ang_inc = 5.0

        
    def cb_joy(self, data):
        isTriggered = false
        self.commands.mode = false # Mode manuel is false

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

        #mode Auto
        if(data.buttons[1]):
            self.commands.mode = true

        if isTriggered: #publish on bras state change from controller input
            self.ang_pub.publish(self.commands)


if __name__=='__main__':
    try:
        bras_t = bras_teleop()
        rospy.init_node('bras_teleop', anonymous=true)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        raise e

