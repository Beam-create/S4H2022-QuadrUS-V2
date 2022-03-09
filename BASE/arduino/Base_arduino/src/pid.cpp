/*
Projet S4 GRO 2022
Class to control Pololu 50:1 Metal Gearmotor 37Dx70L mm 12V 
with 64 CPR Encoder and with Cytron 13Amp 6V-30V DC Motor Driver 
on Arduino Mega 2560
@author Antoine Waltz et al
@version 1.0 09/02/2022
*/

#include "pid.h"

PID::PID(motor &FL_motor, motor &FR_motor, motor &BL_motor, motor &BR_motor){
    _FL_motor = &FL_motor;
    _FR_motor = &FR_motor;
    _BL_motor = &BL_motor;
    _BR_motor = &BR_motor;
}
PID::~PID(){
    
}
void PID::setGains(float kp, float ki, float kd){
    Kp_ = kp; Ki_ = ki; Kd_ = kd;
}
void PID::reset(int motor_id){
    switch(motor_id){
        case 1:
            eIntegral_FL = 0;
            ePrevious_FL = 0;
            lastMeasureTime_FL = millis();
        case 2:
            eIntegral_FR = 0;
            ePrevious_FR = 0;
            lastMeasureTime_FR = millis();
        case 3:
            eIntegral_BL = 0;
            ePrevious_BL = 0;
            lastMeasureTime_BL = millis();
        case 4:
            eIntegral_BR = 0;
            ePrevious_BR = 0;
            lastMeasureTime_BR = millis();
    }
}
void PID::run(float cmd_FL,float cmd_FR,float cmd_BL,float cmd_BR){
    // Serial.println("Entré dans pid.run()");
    if (cmdPrevious_FL != cmd_FL){
        reset(1);
        atGoal_FL = false;
        atGoal_ = false;
        cmdPrevious_FL = cmd_FL;
    }
    else if (cmdPrevious_FR != cmd_FR){
        reset(2);
        atGoal_FR = false;
        atGoal_ = false;
        cmdPrevious_FR = cmd_FR;
    }
    else if (cmdPrevious_BL != cmd_BL){
        reset(3);
        atGoal_BL = false;
        atGoal_ = false;
        cmdPrevious_BL = cmd_BL;
    }
    else if (cmdPrevious_BR != cmd_BR){
        reset(4);
        atGoal_BR = false;
        atGoal_ = false;
        cmdPrevious_BR = cmd_BR;
    }
    // Serial.println("Vérification des commandes faite");
    // variables();
    //if (!atGoal_){
        // Serial.println("if (!atGoal_)");
        //if(!atGoal_FL){
            // Serial.println("if (!atGoal_FL)");
            double FL_error = cmd_FL - _FL_motor->getMeanSpeed();
            // Serial.println(FL_error);
            // Serial.println("Calcul de FL_error");
            Serial.println(FL_error);
            if(fabs(FL_error)<epsilon_){
                // Serial.println("if fabs(FR_error)<epsilon_)");
                atGoal_FL = true;
                _FL_motor->setPWM(cmd_FL);
            }
            else{
                // Serial.println("Entrée dans (*_FL_motor)->setPWM(computeCommand(1,FL_error))");
                _FL_motor->setPWM(computeCommand(1,FL_error));
                // Serial.println("(*_FL_motor)->setPWM(computeCommand(1,FL_error)) Fait!");
            }
        //}
        //if(!atGoal_FR){
            // Serial.println("if (!atGoal_FR)");
            double FR_error = cmd_FR - _FR_motor->getMeanSpeed();
            // Serial.println("Calcul de FR_error");
            // Serial.println(FR_error);
            if(fabs(FR_error)<epsilon_){
                // Serial.println("if fabs(FR_error)<epsilon_)");
                atGoal_FR = true;
                _FR_motor->setPWM(-cmd_FR);
            }
            else{
                _FR_motor->setPWM(computeCommand(2,FR_error));
            }
        //}
        //if(!atGoal_BL){
            // Serial.println("if (!atGoal_BL)");
            double BL_error = cmd_BL - _BL_motor->getMeanSpeed();
            if(fabs(BL_error)<epsilon_){
                atGoal_BL = true;
                _BL_motor->setPWM(cmd_BL);
            }
            else{
                _BL_motor->setPWM(computeCommand(3,BL_error));
            }
        //}
        //if(!atGoal_BR){
            // Serial.println("if (!atGoal_BR)");
            double BR_error = cmd_BR - _BR_motor->getMeanSpeed();
            if(fabs(BR_error)<epsilon_){
                atGoal_BR = true;
                _BR_motor->setPWM(-cmd_BR);
            }
            else{
                _BR_motor->setPWM(computeCommand(4,BR_error));
            }
        //}
        // if(atGoal_FL && atGoal_FR && atGoal_BL && atGoal_BR){
        //     atGoal_ = true;
        // }
    //}
    
}

float PID::computeCommand(int motor_id,double error){
    // Serial.println("computeCommand lancé");
    double eIntegral_;
    switch(motor_id){
        case 1:
            // Serial.println("computeCommand FL_motor");
            eIntegral_FL += error;
            eIntegral_ = eIntegral_FL;
            dt_ = (millis() - lastMeasureTime_FL)/1000.0;
        case 2:
            // Serial.println("computeCommand FR_motor");
            eIntegral_FR += error;
            eIntegral_ = eIntegral_FR;
            dt_ = (millis() - lastMeasureTime_FR)/1000.0;
        case 3:
            // Serial.println("computeCommand BL_motor");
            eIntegral_BL += error;
            eIntegral_ = eIntegral_BL;
            dt_ = (millis() - lastMeasureTime_BL)/1000.0;
        case 4:
            // Serial.println("computeCommand BR_motor");
            eIntegral_BR += error;
            eIntegral_ = eIntegral_BR;
            dt_ = (millis() - lastMeasureTime_BR)/1000.0;
    }
    float CMD;
    
    // Integral saturation
    if(eIntegral_ > eIntegralLim_){
        eIntegral_ = eIntegralLim_;
    }
    if(eIntegral_ < -eIntegralLim_){
        eIntegral_ = -eIntegralLim_;
    }

    switch(motor_id){
        case 1:
            CMD = Kp_*error + Ki_*eIntegral_*dt_ + Kd_*(error-ePrevious_FL)/dt_;
            ePrevious_FL = error;
            lastMeasureTime_FL =  millis();
        case 2:
            CMD = (Kp_*error + Ki_*eIntegral_*dt_ + Kd_*(error-ePrevious_FR)/dt_)*-1;
            ePrevious_FR = error;
            lastMeasureTime_FR =  millis();
        case 3:
            CMD = Kp_*error + Ki_*eIntegral_*dt_ + Kd_*(error-ePrevious_BL)/dt_;
            ePrevious_BL = error;
            lastMeasureTime_BL =  millis();
        case 4:
            CMD = (Kp_*error + Ki_*eIntegral_*dt_ + Kd_*(error-ePrevious_BR)/dt_)*-1;
            ePrevious_BR = error;
            lastMeasureTime_BR =  millis();
    }
    // Serial.println("Sortie de computeCommand");
    // Serial.print("CMD : ");
    // Serial.println(CMD);
    return CMD;
}
void PID::goals(){
    Serial.print("atGoal_ : ");
    Serial.println(this->atGoal_);
    Serial.print("atGoal_FL : ");
    Serial.println(this->atGoal_FL);
    Serial.print("atGoal_FR : ");
    Serial.println(this->atGoal_FR);
    Serial.print("atGoal_BL : ");
    Serial.println(this->atGoal_BL);
    Serial.print("atGoal_BR : ");
    Serial.println(this->atGoal_BR);
}

