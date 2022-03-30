#!/usr/bin/env python
"""
File to convert joy messages to joint angles messages
Subscriber : joy
Publisher : rufus/bras_teleop
"""

import rospy
from sensor_msgs.msg import Joy
from rufus_master.msg import bras_commands

class bras_teleop:

    def __init__(self):
        """
        Initialize subscriber, pulisher and node
        """
        #objet message de commande
        self.commands = bras_commands()

        self.joy_sub = rospy.Subscriber("joy", Joy, self.cb_joy)
        self.ang_pub = rospy.Publisher("rufus/bras_arduino", bras_commands, queue_size=5)

        # Initial values of angles on start-up
        self.commands.q1 = 0.0
        self.commands.q2 = 90.0
        self.commands.q3 = 0.0
        self.commands.mode = False
        self.commands.effector = False

        self.ang_inc = 5.0

        self.lim = {
            "q1_min":-45.0,
            "q1_max":45.0,
            "q2_min":30.0,
            "q2_max":130.0,
            "q3_min":-15.0,
            "q3_max":60.0
        }

        
    def cb_joy(self, data):
        isTriggered = False

        # Tester config manette pour attribuer les valeurs a angles.q*
        #q1
        if(data.buttons[16]):
            self.commands.q1 = self.commands.q1 + self.ang_inc
            isTriggered = True
            self.commands.mode = False # Mode manuel is false
        if(data.buttons[15]):
            self.commands.q1 = self.commands.q1 - self.ang_inc
            isTriggered = True
            self.commands.mode = False # Mode manuel is false

        #q2
        if(data.buttons[13]):
            self.commands.q2 = self.commands.q2 + self.ang_inc
            isTriggered = True
            self.commands.mode = False # Mode manuel is false
        if(data.buttons[14]):
            self.commands.q2 = self.commands.q2 - self.ang_inc
            isTriggered = True
            self.commands.mode = False # Mode manuel is false

        #q3
        if(data.buttons[0]):
            self.commands.q3 = self.commands.q3 + self.ang_inc
            isTriggered = True
            self.commands.mode = False # Mode manuel is false
        if(data.buttons[2]):
            self.commands.q3 = self.commands.q3 - self.ang_inc
            isTriggered = True
            self.commands.mode = False # Mode manuel is false

        #effector
        if(data.buttons[5]):
            self.commands.effector = True
            isTriggered = True
            self.commands.mode = False # Mode manuel is false
        if(data.buttons[4]):
            self.commands.effector = False
            isTriggered = True
            self.commands.mode = False # Mode manuel is false


        # Go to home
        if(data.buttons[8]):
            self.commands.q1 = 0.0
            self.commands.q2 = 90.0
            self.commands.q3 = 0.0
            isTriggered = True

        #mode Auto
        if(data.buttons[1]):
            self.commands.mode = True
            isTriggered = True

        if isTriggered: #publish on bras state change from controller input
            self.ang_pub.publish(self.commands)


if __name__=='__main__':
    bras_t = bras_teleop()
    rospy.init_node('bras_teleop', anonymous=True)
    rospy.spin()


