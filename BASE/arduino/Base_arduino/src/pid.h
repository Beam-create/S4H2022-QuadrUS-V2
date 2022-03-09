/*
Projet S4 GRO 2022
Class to control Pololu 50:1 Metal Gearmotor 37Dx70L mm 12V 
with 64 CPR Encoder and with Cytron 13Amp 6V-30V DC Motor Driver 
on Arduino Mega 2560
@author Antoine Waltz et al
@version 1.0 09/02/2022
*/

#ifndef pid_H_
#define pid_H_

#include <Arduino.h>
#include <math.h>
#include "motor.h"

class PID{
    public:
        PID(motor &FL_motor, motor &FR_motor, motor &BL_motor, motor &BR_motor);
        ~PID();
        void setGains(float kp, float ki, float kd);
        void reset(int motor_id);
        void run(float cmd_FL,float cmd_FR,float cmd_BL,float cmd_BR);
        float computeCommand(int motor_id,double error);
        bool goal(){return atGoal_;};
        void goals();

        motor *_FL_motor; // 1
        motor *_FR_motor; // 2
        motor *_BL_motor; // 3
        motor *_BR_motor; // 4
    private:

        bool atGoal_ = false; // Flag to know if at goal
        bool atGoal_FL = false;
        bool atGoal_FR = false;
        bool atGoal_BL = false;
        bool atGoal_BR = false;

        double Kp_ = 1.0; // Proportional constant
        double Ki_ = 1.0; // Integral constant
        double Kd_ = 1.0; // Derivative constant
        double dt_; // Theoric time between 2 measurments
        unsigned long lastMeasureTime_FL = 0; // Time of last iteration
        unsigned long lastMeasureTime_FR = 0; // Time of last iteration 
        unsigned long lastMeasureTime_BL = 0; // Time of last iteration 
        unsigned long lastMeasureTime_BR = 0; // Time of last iteration 

        double epsilon_ = 0.05;
        double eIntegralLim_ = 100;
        double eIntegral_FL = 0; // Variable to store the sum of errors
        double eIntegral_FR = 0; // Variable to store the sum of errors
        double eIntegral_BL = 0; // Variable to store the sum of errors
        double eIntegral_BR = 0; // Variable to store the sum of errors
        double ePrevious_FL = 0; // Variable to store the last error
        double ePrevious_FR = 0; // Variable to store the last error
        double ePrevious_BL = 0; // Variable to store the last error
        double ePrevious_BR = 0; // Variable to store the last error
        float cmdPrevious_FL = 0;
        float cmdPrevious_FR = 0;
        float cmdPrevious_BL = 0;
        float cmdPrevious_BR = 0;
};

#endif //pid