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
        self.ang_sub = rospy.Subscriber("rufus/bras_arduino", bras_commands, self.cb_sync)

        self.ang_pub = rospy.Publisher("rufus/bras_arduino", bras_commands, queue_size=1)

        # Initial values of angles on start-up
        self.commands.q1 = 0.0
        self.commands.q2 = 90.0
        self.commands.q3 = 0.0
        self.commands.gimbalAng = 90.0
        self.commands.mode = False
        self.commands.effector = False
        self.commands.IK = False

        self.ang_inc = 0.5

        self.flags = {
            "q1+": False,
            "q1-": False,
            "q2+": False,
            "q2-": False,
            "q3+": False,
            "q3-": False,
            "gim+": False,
            "gim-": False,
        }

        self.lim = {
            "q1_min":-45.0,
            "q1_max":45.0,
            "q2_min":30.0,
            "q2_max":130.0,
            "q3_min":-15.0,
            "q3_max":60.0,
            "gim_max":90.0,
            "gim_min":20.0
        }

    def cb_sync(self, data):
        if data.IK:
            self.commands.mode = data.mode
            self.commands.IK = data.IK
            self.commands.q1 = data.q1
            self.commands.q2 = data.q2
            self.commands.q3 = data.q3
        
    def cb_joy(self, data):
        # Tester config manette pour attribuer les valeurs a angles.q*
        # Force set au mode manuelle en cas dappuis
        if (data.buttons[16] or data.buttons[15] or data.buttons[13] or data.buttons[14] or data.buttons[0] or data.buttons[2] or data.buttons[5] or data.buttons[4] or data.buttons[8]):
            self.commands.mode = False
            self.commands.IK = False

        #q1
        self.flags["q1+"] = True if data.buttons[16] else False
        self.flags["q1-"] = True if data.buttons[15] else False

        #q2
        self.flags["q2+"] = True if data.buttons[13] else False
        self.flags["q2-"] = True if data.buttons[14] else False

        #q3
        self.flags["q3+"] = True if data.buttons[0] else False
        self.flags["q3-"] = True if data.buttons[2] else False


        #gimbal
        self.flags["gim+"] = True if data.buttons[7] else False
        self.flags["gim-"] = True if data.buttons[6] else False

        #effector
        if(data.buttons[5]):
            self.commands.effector = True
        if(data.buttons[4]):
            self.commands.effector = False


        # Go to home
        if(data.buttons[8]):
            self.commands.q1 = 0.0
            self.commands.q2 = 90.0
            self.commands.q3 = 0.0

        #mode Auto
        if(data.buttons[1]):
            self.commands.mode = True

    def controllerCommands(self):
        # Fonction d'envoie de parametre a 10Hz 
        # q1
        if(self.flags["q1+"]):
            self.commands.q1 = self.lim["q1_max"] if (self.commands.q1 + self.ang_inc >= self.lim["q1_max"]) else self.commands.q1 + self.ang_inc
        if(self.flags["q1-"]):
            self.commands.q1 = self.lim["q1_min"] if (self.commands.q1 - self.ang_inc <= self.lim["q1_min"]) else self.commands.q1 - self.ang_inc

        #q2
        if(self.flags["q2+"]): #La verif des limites se fait 
            self.commands.q2 = self.commands.q2 + self.ang_inc
        if(self.flags["q2-"]):
            self.commands.q2 = self.commands.q2 - self.ang_inc
        
        #q3
        if(self.flags["q3+"]):
            self.commands.q3 = self.commands.q3 + self.ang_inc
        if(self.flags["q3-"]):
            self.commands.q3 = self.commands.q3 - self.ang_inc
        
        # #Gimbal control
        if(self.flags["gim+"]):
            self.commands.gimbalAng = self.lim["gim_max"] if (self.commands.gimbalAng + self.ang_inc >= self.lim["gim_max"]) else self.commands.gimbalAng + self.ang_inc
        if(self.flags["gim-"]):
            self.commands.gimbalAng = self.lim["gim_min"] if (self.commands.gimbalAng - self.ang_inc <= self.lim["gim_min"]) else self.commands.gimbalAng - self.ang_inc
        
if __name__=='__main__':
    try:
        bras_t = bras_teleop()
        rospy.init_node('bras_teleop', anonymous=True)
        rate = rospy.Rate(22)
        while not rospy.is_shutdown():
            bras_t.controllerCommands()
            bras_t.ang_pub.publish(bras_t.commands)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass


