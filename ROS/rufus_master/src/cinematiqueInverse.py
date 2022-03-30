#!/usr/bin/env python3
import sys
from time import sleep
from click import echo
from sympy import *
import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from rufus_master.msg import bras_commands
sys.path.append('~/rufus_ws/src/S4H2022-projet/ROS/rufus_master/src')
from Limits import *

#Fonction pour la generation des angles des joints en fonction de positions cartesienne

class robotArm:

	L1 = 0.095 #m
	L2 = 0.160 #m
	L3 = 0.180 #m
	L4y = 0.098 #m
	L4x = 0.035 #m


    #Changer angle initiale pour les bonnes valeurs
	def __init__(self, initial_q1 = 0.0, initial_q2 = 90.0, initial_q3 = 0.0):
		self.q = bras_commands()
		self.q.q1 = initial_q1
		self.q.q2 = initial_q2
		self.q.q3 = initial_q3
		self.mode = False
		self.IK = False

		self.ball_position = Vector3()
		#Subcribe to the ball position publish by limits
		self.sub = rospy.Subscriber("/rufus/ball_position", Vector3, self.cb_camera)
		#Subscribe to the remote
		self.command = rospy.Subscriber("rufus/bras_arduino", bras_commands, self.start)

		#Publisher
		self.pub = rospy.Publisher("rufus/bras_arduino",bras_commands, queue_size = 1)
		
	def cb_camera(self, data):
		if verify_limits([data.x, data.y, data.z], 1):
			self.ball_position = data
		else:
			print("Flag 1")

	def start(self, data):
		self.mode = data.mode
		self.IK = data.IK
			
            
	def inverseKinematics(self, vector_pos):
		"""
		Fonction to fin the Inverse Kinematics for robot arm
		:param x: Position of the object on the 'x' axis
		:param y: Position of the object on the 'y' axis
		:param z: Position of the object on the 'z' axis
		:return: q1, q2, q3 -> Corresponding joint angles in degrees
		"""
		L1 = self.L1
		L2 = self.L2
		L3 = self.L3
		L4y = self.L4y
		L4x = self.L4x
		x = vector_pos.x
		y = vector_pos.y
		z = vector_pos.z
		pi = 3.14159265359
        
		# Find the value for the first angle
		q1 = atan2(z, x)
		Angle_q1 = round(float(q1)*180 / pi, 2)

		#Solving equation for q2 and q3
		a = Symbol('a') # Angle q2
		b = Symbol('b') # Angle q3
		
		########## Solution finale pour la resolution de la cinematique inverse #############
		e1 = Eq(cos(q1)*(L2*cos(a) + L3*cos(b) + L4x) - x, 0.0)
		e2 = Eq(0.07695 + L1 + L2*sin(a) - L3*sin(b) - L4y - y, 0.0)
		sol = solve([e1, e2], [a, b])
		print("Flag 2")
        
        ## Choix de l'angle positif
		if float(sol[0][0]) < 0.0:
			Angle_q2 = round(float(sol[1][0]) * 180 / pi, 2)
		if float(sol[1][0]) < 0.0:
			Angle_q2 = round(float(sol[0][0]) * 180 / pi, 2)
		if float(sol[0][1]) < 0.0:
			Angle_q3 = round(float(sol[1][1]) * 180 / pi, 2)
		if float(sol[1][1]) < 0.0:
			Angle_q3 = round(float(sol[0][1]) * 180 / pi, 2)

		self.q.q1 = round(Angle_q1,2)
		self.q.q2 = round(Angle_q2,2)
		self.q.q3 = round(Angle_q3,2)
		self.q.mode = True
		    
           
if __name__=='__main__':
	try:
		r_a = robotArm()
		rospy.init_node('robot_control', anonymous=True)
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			if r_a.mode and r_a.IK:
				r_a.inverseKinematics(r_a.ball_position)
				r_a.pub.publish(r_a.q)
			rate.sleep()
	except rospy.ROSInterruptException:
		pass
	
	


