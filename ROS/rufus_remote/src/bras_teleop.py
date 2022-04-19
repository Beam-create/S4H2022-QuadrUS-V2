#!/usr/bin/env python3
"""
File to convert joy messages to joint angles messages
Subscriber : joy
Publisher : rufus/bras_teleop
"""

import rospy
from sensor_msgs.msg import Joy
from rufus_master.msg import bras_commands
from geometry_msgs.msg import Vector3
from sympy import *

class bras_teleop:

    def __init__(self):
        """
        Initialize subscriber, pulisher and node
        """
        #objet message de commande
        self.commands = bras_commands()

        self.joy_sub = rospy.Subscriber("joy", Joy, self.cb_joy)
        self.cam_sub = rospy.Subscriber("/camera/Ball_pos", Vector3, self.cb_cam)

        self.comm_pub = rospy.Publisher("rufus/bras_arduino", bras_commands, queue_size=1)

        # Initial values of bras_command message
        self.commands.q1 = 0.0
        self.commands.q2 = 90.0
        self.commands.q3 = 0.0
        self.commands.gimbalAng = 90.0
        self.commands.mode = False
        self.commands.effector = False

        self.L1 = 9.5 #cm
        self.L2 = 16.0 #cm
        self.L3 = 18.0 #cm
        self.L4y = 9.8 #cm
        self.L4x = 3.5 #cm
        self.camx = 12.48
        self.camy = 10.87

        self.isGood = False
        self.ball_position = [0] * 3

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
            "x_min":0.0,
            "x_max":200.0,
            "y_min":0.0,
            "y_max":200.0,
            "z_min":-200.0,
            "z_max":200.0,


            "q1_min":-45.0,
            "q1_max":45.0,
            "q2_min":30.0,
            "q2_max":130.0,
            "q3_min":-15.0,
            "q3_max":60.0,
            "gim_max":90.0,
            "gim_min":20.0
        }


##################### Class methods #######################
    def verify_camLimits(self, array):

        if array[0] <= self.lim['x_min'] or array[0] >= self.lim['x_max'] or array[1] <= self.lim['y_min'] or array[1] >= self.lim['y_max'] or array[2] <= self.lim['z_min'] or array[2] >= self.lim['z_max']:
            self.isGood = False
            return False
        else:
            self.isGood = True
            return True


    def inverseKinematics(self):
        """
        Fonction to fin the Inverse Kinematics for robot arm
        :param x: Position of the object on the 'x' axis
        :param y: Position of the object on the 'y' axis
        :param z: Position of the object on the 'z' axis
        :return: q1, q2, q3 -> Corresponding joint angles in degrees
        """

        x = self.ball_position.x
        y = self.ball_position.y
        z = self.ball_position.z
        pi = 3.14159265359

        ik_angles = [0] * 3 # Init empty array of size 3

        # Find the value for the first angle
        q1 = atan2(z, x)


        #Solving equation for q2 and q3
        a = Symbol('a') # Angle q2
        b = Symbol('b') # Angle q3

        ########## Solution finale pour la resolution de la cinematique inverse #############
        e1 = Eq(cos(q1)*(self.L2*cos(a) + self.L3*cos(b) + self.L4x) - x - self.camx, 0.0) #x equation
        e2 = Eq(self.camy + self.L1 + self.L2*sin(a) - self.L3*sin(b) - self.L4y - y, 0.0) #y equation
        sol = nsolve([e1, e2], [a, b], [pi/2, 0]) #pi/2 rad est l'angle initiale q2 et 0 rad est q3

        Angle_q2 = float(sol[0])*180/pi
        Angle_q3 = float(sol[1])*180/pi

        #Angles mit en deg.
        ik_angles[0] = round(float(q1)*180 / pi, 2)
        ik_angles[1] = round(Angle_q2,2)
        ik_angles[2] = round(Angle_q3,2)
        return ik_angles


    ################## Callback functions ###################
    def cb_cam(self, data):
        """
        Fonction callback from camera topic and verifies limits of received message
        :param x: Position of the object on the 'x' axis
        :param y: Position of the object on the 'y' axis
        :param z: Position of the object on the 'z' axis
        :return: void
        """
        if self.verify_camLimits([data.x, data.y, data.z]):
            self.ball_position = data

    def cb_joy(self, data):
        # Tester config manette pour attribuer les valeurs a angles.q*
        # Force set au mode manuelle en cas dappuis
        if (data.buttons[8]):#data.buttons[16] or data.buttons[15] or data.buttons[13] or data.buttons[14] or data.buttons[0] or data.buttons[2] or data.buttons[5] or data.buttons[4] or 
            self.commands.mode = False

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
        if(data.buttons[1] and self.isGood):
            self.commands.mode = True

    def controllerCommands(self):
        """
        Fonction to set command message according to user inputs
        :return: void
        """

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
        
        # Gimbal control
        if(self.flags["gim+"]):
            self.commands.gimbalAng = self.lim["gim_max"] if (self.commands.gimbalAng + self.ang_inc >= self.lim["gim_max"]) else self.commands.gimbalAng + self.ang_inc
        if(self.flags["gim-"]):
            self.commands.gimbalAng = self.lim["gim_min"] if (self.commands.gimbalAng - self.ang_inc <= self.lim["gim_min"]) else self.commands.gimbalAng - self.ang_inc

        # IK mode
        if(self.commands.mode):
            try:
                Angles = self.inverseKinematics()
                self.commands.q1 = Angles[0]
                self.commands.q2 = Angles[1]
                self.commands.q3 = Angles[2]
            except:
                pass
        
if __name__=='__main__':
    # Messages are published at a rate of 22Hz to bras_commands
    try:
        bras_t = bras_teleop()
        rospy.init_node('bras_teleop', anonymous=True)
        rate = rospy.Rate(22)
        while not rospy.is_shutdown():
            bras_t.controllerCommands()
            bras_t.comm_pub.publish(bras_t.commands)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass


