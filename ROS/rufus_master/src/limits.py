#!/usr/bin/env python
from logging import raiseExceptions
import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool

class limits:

    angleLimits = {
        "joint1_max": 45,
        "joint1_min": -45,
        "joint2_max": 130,
        "joint2_min": 40,
        "joint3_max": 40,
        "joint3_min": -10
    }
    reachLimits = {
        "x_max": 1,
        "x_min": 2,
        "y_max": 3,
        "y_min": 4,
        "z_max": 5,
        "z_min": 6
    }


    def __init__(self):
        self.pick = False
        self.good_position = False
        self.ball_position = Vector3()
        #Subscribe to the camera topic # Check if the ball is reacheable
        self.limits_sub = rospy.Subscriber("/rufus/camera/ball_position", Vector3, self.checkReachLimits)
        #Subscribe to see if pick ball is done
        self.pick_status = rospy.Subscriber("/rufus/state_pick", Bool, self.checkState)
        
        #Publish to topic /rufus/ball_position if ball reacheable
        self.limits_pub = rospy.Publisher("/rufus/ball_position", Vector3, queue_size=1)

    def checkState(self,state):
        if state: #If True, the ball is pick and mvt finish
            self.pick = True
        else: #When false we do not publish in topic /rufus/ball_position because compute in progress
            self.pick = False
            

    def checkErrorJointLimits(self, angle_q1, angle_q2, angle_q3):
        """
		Function : Check if the joints angles are in range
		:param angle_q1:
		:param angle_q2:
		:param angle_q3:
		:return: If outside of joints limits return error code for appropriate joint
		Otherwise return zero value
		"""
        if angle_q1 < self.angleLimits["joint1_min"] or angle_q1 > self.angleLimits["joint1_max"] :
            raise Exception('Angle value joint 1 out of limits')
        if angle_q2 < self.angleLimits["joint2_min"]  or angle_q2 > self.angleLimits["joint2_max"] :
            raise Exception('Angle value joint 2 out of limits')
        if angle_q3 < self.angleLimits["joint3_min"]  or angle_q3 > self.angleLimits["joint3_max"] :
            raise Exception('Angle value joint 3 out of limits')
        else:
            return True

    def checkReachLimits(self, vector_position):
        """
        Fonction to evaluate if the object is reacheable
        :param x: Position of the object on the 'x' axis
        :param y: Position of the object on the 'y' axis
        :param z: Position of the object on the 'z' axis
        :return: If the object is not reacheable, return error code for the appropriate axis
        Otherwise return zero value
        """
        x = vector_position.x
        y = vector_position.y
        z = vector_position.z

        if x > self.reachLimits["x_max"] or x < self.reachLimits["x_min"] or y > self.reachLimits["y_max"]  or y < self.reachLimits["y_min"] or z > self.reachLimits["z_max"]  or z < self.reachLimits["z_min"]:
            self.good_position = False
            raise Exception('Position of the ball if out of limits')
        else:
            self.good_position = True
            self.ball_position = vector_position

if __name__ == '__main__':
    try:
        lim = limits()
        rospy.init_node('LIMITS', anonymous=True)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if lim.good_position and lim.pick:
                lim.limits_pub.publish(lim.ball_position)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

    