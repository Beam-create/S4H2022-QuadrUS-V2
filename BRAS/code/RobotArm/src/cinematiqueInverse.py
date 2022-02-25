import math
from sympy import symbols, Eq, solve, cos, sin

#Fonction pour la generation des angles des joints en fonction de positions cartesienne

class robotArm:
    q1_max_min = [3,0]
    q2_max_min = [3,0]
    q3_max_min = [3,0]
    x_max_min = [10,20]
    y_max_min = [10,20]
    z_max_min = [10,20]
    L1 = 0.092 #m
    L2 = 0.135 #m
    L3 = 0.147 #m
    L4 = 0.087 #m


    def __init__(self, initial_q1 = 0, initial_q2 = 0, initial_q3 = 0):
        self.q1 = initial_q1
        self.q2 = initial_q2
        self.q3 = initial_q3

    def checkErrorJointLimits(self, angle_q1, angle_q2, angle_q3):
        if angle_q1 < self.q1_max_min[2] or angle_q1 > self.q3_max_min[1]:
            raise Exception('Angle value joint 1 out of limits')
        if angle_q2 < self.q2_max_min[2] or angle_q2 > self.q2_max_min[1]:
            raise Exception('Angle value joint 2 out of limits')
        if angle_q3 < self.q3_max_min[2] or angle_q3 > self.q3_max_min[1]:
            raise Exception('Angle value joint 3 out of limits')

    def checkReachLimits(self, x, y, z):
        if x > self.x_max_min[1] or x < self.x_max_min[2]:
            raise Exception('Position in x axis out of limits')
        if y > self.y_max_min[1] or y < self.y_max_min[2]:
            raise Exception('Position in y axis out of limits')
        if z > self.z_max_min[1] or z < self.z_max_min[2]:
            raise Exception('Position in z axis out of limits')

    def inverseKinematics(self, x, y, z):
        L1 = self.L1
        L2 = self.L2
        L3 = self.L3
        L4 = self.L4

        #Reach position vector
        r = math.sqrt(x**2 + y**2)

        #Position of end joint 3 (Joint 4)
        y_joint4 = y + L4 + L1
        x_joint4 = x

        #position of joint 1
        y_joint1 = L1
        x_joint1 = 0

        #distance joint 1 to joint 4(fix)
        r_xy_4 = math.sqrt(x_joint4**2 + y_joint4**2)

        #Find the value for the first angle
        Angle_q1 = math.atan2(x,z)

        #Find angle q3
        Angle_q3 = -math.acos((L2**2 + L3**2 - r_xy_4**2)/(2*L2*L3))
        #Angle_q3 = math.pi - alpha

        #Find angle q2
        L3_sin_q3 = L3*math.sin(Angle_q3)
        L3_cos_q3 = L3*math.cos(Angle_q3)
        beta = math.atan2(L3_sin_q3, L2+L3_cos_q3)
        Angle_q2 = math.atan2(y_joint4, x) + beta

        #Solving equation for q2 and q3
        q2 = symbols('q2')
        q3 = symbols('q3')
        # e1 = Eq(L1 + L2*cos(q2)+L3*cos(q2+q3)-L4 - y, 0)
        # e2 = Eq(-L2*sin(q2) - L3*sin(q2+q3) - x, 0)
        # sol = solve((e1,e2),(q2,q3))

        # Angle_q2 = q2
        # Angle_q3 = q3
        # print(sol)

        print(L1, L2, L3, L4)
        print("Angle q2 =", Angle_q2)
        print("Angle q3 =",Angle_q3)


        return Angle_q1, Angle_q2, Angle_q3




