#!/usr/bin/env python
"""
Noeud pour convertir les messages de vitesses (twist) en commandes moteur
Subscriber : rufus/base_teleop
Publisher : rufus/base_arduino
"""

from sympy import true
import rospy
from geometry_msgs.msg import Twist
from rufus_master.msg import Rufus_base_msgs
import math

class base_control:

    def __init__(self):
        self.motor_cmd = Rufus_base_msgs()
        self.data = Twist()

        self.twist_sub = rospy.Subscriber("rufus/base_teleop", Twist, self.cb)
        self.arduino_pub = rospy.Publisher("/rufus/base_arduino", Rufus_base_msgs, queue_size=200)

    def cb(self,data):
        self.vector_to_motor(data)
        self.arduino_pub.publish(self.motor_cmd)
        self.data = data

    def vector_to_motor(self, vector):
        """
        Calcul de la commande vitesse de chaque moteur
        """
        #Définir le vecteur vitesse
        power = math.hypot(vector.linear.y, vector.linear.x)
        angle = math.atan2(vector.linear.x, vector.linear.y)
        turn = vector.angular.z

        #Définir les composantes
        sin = math.sin(angle - math.pi/4)
        cos = math.cos(angle - math.pi/4)
        comp_list = [abs(sin), abs(cos)]
        max_comp = max(comp_list)

        #Calculer la commande pour chaque moteurs
        self.motor_cmd.motor_FL = power * cos/max_comp + turn
        self.motor_cmd.motor_FR = -1*(power * sin/max_comp - turn)
        self.motor_cmd.motor_BL = power * sin/max_comp + turn
        self.motor_cmd.motor_BR = -1*(power * cos/max_comp - turn)

        #Normaliser les commandes entre 1 et -1
        if((power + abs(turn)) > 1):
            self.motor_cmd.motor_FL /= (power + abs(turn))
            self.motor_cmd.motor_FR /= (power + abs(turn))
            self.motor_cmd.motor_BL /= (power + abs(turn))
            self.motor_cmd.motor_BR /= (power + abs(turn))

if __name__=='__main__':
    try:
        b_c = base_control()
        
        rospy.init_node('base_control', anonymous=true)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            b_c.vector_to_motor(b_c.data)
            b_c.arduino_pub.publish(b_c.motor_cmd)
            rate.sleep()
    
    except rospy.ROSInterruptException:
        pass