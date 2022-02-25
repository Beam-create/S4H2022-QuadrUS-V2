/*
Projet S4 GRO 2022
Class to control Pololu 50:1 Metal Gearmotor 37Dx70L mm 12V 
with 64 CPR Encoder and with Cytron 13Amp 6V-30V DC Motor Driver 
on Arduino Mega 2560
@author Antoine Waltz et al
@version 1.0 09/02/2022
*/

#include "pid.h"

PID::PID(motor FL_motor, motor FR_motor, motor BL_motor, motor BR_motor){
    _FL_motor = FL_motor;
    _FR_motor = FR_motor;
    _BL_motor = BL_motor;
    _BR_motor = BR_motor;
}
PID::~PID(){
    
}
// void PID::run(int motor_number, float speed){
//     float error = 1.0;
//     float command = 0.0;
//     motor runMotor;
//     switch (motor_number){
//         case 1:
//             runMotor = this->_FL_motor;
//         case 2:
//             runMotor = this->_FR_motor;
//         case 3:
//             runMotor = this->_BL_motor;
//         case 4:
//             runMotor = this->_BR_motor;
//     }
//     while (error > 0.05){
//         error = abs(speed - runMotor.getMeanSpeed());
//         // command = kp + ...;
//     }

// }

void PID::run(){
    // if enabled and time to run iteration
    if(millis() >= measureTime_ && enable_){

        //actualDt_ = millis() - measureTime_;
        measureTime_ = millis() + dtMs_;
        double error = goal_ - measurementFunc_();
        // goal_ = input speed, measurementFunc_ = motor.getMeanSpeed()
        
        // if goal reached
        if(fabs(error)<epsilon_){

            atGoal_ = true;
            //enable_ = false;
            if (atGoalFunc != nullptr){
                atGoalFunc();
            }
        }else{
            commandFunc_(computeCommand(error));
        }
       lastMeasureTime_ =  measureTime_;
    }

}

double PID::computeCommand(double error){
    double CMD;
    actualDt_ = millis() - lastMeasureTime_;
    eIntegral_ += error;
    
    // Integral saturation
    if(eIntegral_ > eIntegralLim_){
        eIntegral_ = eIntegralLim_;
    }
    if(eIntegral_ < -eIntegralLim_){
        eIntegral_ = -eIntegralLim_;
    }

    CMD = Kp_*error + Ki_*eIntegral_*dt_ + Kd_*(error-ePrevious)/dt_;

    ePrevious = error;
    return CMD;
}

