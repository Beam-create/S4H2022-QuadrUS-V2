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
        PID(motor FL_motor, motor FR_motor, motor BL_motor, motor BR_motor);
        ~PID();
        void run();
        double computeCommand(double error);
    private:
        motor _FL_motor; // 1
        motor _FR_motor; // 2
        motor _BL_motor; // 3
        motor _BR_motor; // 4

        // Function pointers
        double (*measurementFunc_)() = nullptr; // Measurement function
        void (*commandFunc_)(double) = nullptr; // Command function
        void (*atGoalFunc)() = nullptr; // Fonction called when goal is reached

        double goal_ = 0; // Desired state
        bool enable_ = false; // Enable flag
        bool atGoal_ = false; // Flag to know if at goal 

        double Kp_ = 1.0; // Proportional constant
        double Ki_ = 1.0; // Integral constant
        double Kd_ = 1.0; // Derivative constant
        double dt_; // Theoric time between 2 measurments
        unsigned long dtMs_; // Periode between commands
        unsigned long actualDt_; // Actual periode between last command
        unsigned long measureTime_ = 0; // Time for next iteration 
        unsigned long lastMeasureTime_ = 0; // Time of last iteration 

        double epsilon_ = 5;
        double eIntegralLim_ = 100;
        double eIntegral_ = 0; // Variable to store the sum of errors
        double ePrevious = 0; // Variable to store the last error
};

#endif //pid