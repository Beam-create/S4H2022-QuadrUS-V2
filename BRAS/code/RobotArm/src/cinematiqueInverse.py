
from sympy import *

#Fonction pour la generation des angles des joints en fonction de positions cartesienne

class robotArm:
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
    L1 = 0.095 #m
    L2 = 0.160 #m
    L3 = 0.180 #m
    L4y = 0.098 #m
    L4x = 0.035 #m


    def __init__(self, initial_q1 = 0, initial_q2 = 0, initial_q3 = 0):
        self.q1 = initial_q1
        self.q2 = initial_q2
        self.q3 = initial_q3

    def checkErrorJointLimits(self, angle_q1, angle_q2, angle_q3):

        """
        Function : Check if the joints angles are in range

        :param angle_q1:
        :param angle_q2:
        :param angle_q3:
        :return: If outside of joints limits return error code for appropriate joint
        Otherwise return zero value
        """
        if angle_q1 < self.angleLimits["joint1_min"] or angle_q1 > self.angleLimits["joint1_max"]:
            raise Exception('Angle value joint 1 out of limits')
        if angle_q2 < self.angleLimits["joint2_min"]  or angle_q2 > self.angleLimits["joint2_max"] :
            raise Exception('Angle value joint 2 out of limits')
        if angle_q3 < self.angleLimits["joint3_min"]  or angle_q3 > self.angleLimits["joint3_max"] :
            raise Exception('Angle value joint 3 out of limits')
        else:
            return True

    def checkReachLimits(self, x, y, z):
        """
        Fonction to evaluate if the object is reacheable

        :param x: Position of the object on the 'x' axis
        :param y: Position of the object on the 'y' axis
        :param z: Position of the object on the 'z' axis
        :return: If the object is not reacheable, return error code for the appropriate axis
        Otherwise return zero value
        """
        if x > self.reachLimits["x_max"] or x < self.reachLimits["x_min"]:
            raise Exception('Position in x axis out of limits')
        if y > self.reachLimits["y_max"]  or y < self.reachLimits["y_min"] :
            raise Exception('Position in y axis out of limits')
        if z > self.reachLimits["z_max"]  or z < self.reachLimits["z_min"] :
            raise Exception('Position in z axis out of limits')

    def inverseKinematics(self, x, y, z):

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

        pi = 3.14159265359

        # Find the value for the first angle
        q1 = atan2(z, x)
        Angle_q1 = round(q1*180/pi,2)

        #Solving equation for q2 and q3
        a = Symbol('a') # Angle q2
        b = Symbol('b') # Angle q3

        ########## Solution finale pour la resolution de la cinematique inverse #############
        e1 = Eq(cos(q1)*(L2*cos(a) + L3*cos(b) + L4x) - x, 0.0)
        e2 = Eq(L1 + L2*sin(a) - L3*sin(b) - L4y - y, 0.0)
        # e3 = Eq(cos(q1)*(-L2*sin(a)-L3*sin(b)), 0.0) # Equation diff de e1
        # e4 = Eq(L3*cos(b), L2*cos(a)) # Equation diff de e2
        sol = solve([e1, e2], [a, b])

        print(sol)
        print("Angle 1 =", Angle_q1)
        print("Theta 2 choix 1 =", round(float(sol[0][0]) * 180 / pi, 2))
        print("Theta 2 choix 2 =", round(float(sol[1][0]) * 180 / pi, 2))
        print("Theta 3 choix 1 =", round(float(sol[0][1]) * 180 / pi, 2))
        print("Theta 3 choix 2 =", round(float(sol[1][1]) * 180 / pi, 2))

        ## Choix de l'angle positif
        if float(sol[0][0]) < 0.0:
            Angle_q2 = round(float(sol[1][0]) * 180 / pi, 2)
        if float(sol[1][0]) < 0.0:
            Angle_q2 = round(float(sol[0][0]) * 180 / pi, 2)
        if float(sol[0][1]) < 0.0:
            Angle_q3 = round(float(sol[1][1]) * 180 / pi, 2)
        if float(sol[1][1]) < 0.0:
            Angle_q3 = round(float(sol[0][1]) * 180 / pi, 2)

        if self.checkErrorJointLimits(Angle_q1, Angle_q2, Angle_q3) :
            return Angle_q1, Angle_q2, Angle_q3
        else:
            return False




