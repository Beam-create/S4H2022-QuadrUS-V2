
/*
Projet S4 GRO 2022
Class to control Pololu 50:1 Metal Gearmotor 37Dx70L mm 12V 
with 64 CPR Encoder and with Cytron 13Amp 6V-30V DC Motor Driver 
on Arduino Mega 2560
@author Antoine Waltz et al
@version 1.0 09/02/2022
*/

#ifndef motor_H_
#define motor_H_

#include <Arduino.h>
#include <math.h>

class motor{
    public:
        motor();
        motor(int ENC_A_pin, int ENC_B_pin, int DIR_pin, int PWM_pin);
        ~motor();
        void init();
        bool checkPins(int pin, int arg_nb);
        bool safeCheck();
        // void setPin(char _pinName[11],int pinValue);
        int getPin(char _pinName[11]);
        void getPins();
        void readEncoder();
        void resetEncoder();
        int getEncoderPos();
        int getNbRotation();
        int getEncoderPosTotal();
        int getResetEncoderPos();
        void setPWM(float speed);
        double getMeanSpeed();
        char _ENC_A[11] = "_ENC_A_pin";
        char _ENC_B[11] = "_ENC_B_pin";
        char _DIR[11] = "_DIR_pin";
        char _PWM[11] = "_PWM_pin";
    protected:
        int _ENC_A_pin;
        int _ENC_B_pin;
        int _DIR_pin;
        int _PWM_pin;
        int encoder_pos = 0;
        int nb_rotation = 0;
        int encoder_max_pos = 800;
        int mean_time = 50;
        float set_speed = 0;
        float pulse_per_seconds_to_unit = 0.5; // TBD
};

#endif //motor