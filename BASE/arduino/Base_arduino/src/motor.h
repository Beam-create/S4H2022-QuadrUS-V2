
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
        char _ENC_A[11] = "_ENC_A_pin"; // Noms associés aux pins
        char _ENC_B[11] = "_ENC_B_pin";
        char _DIR[11] = "_DIR_pin";
        char _PWM[11] = "_PWM_pin";
    protected:
        int _ENC_A_pin; // Pins de l'encodeur
        int _ENC_B_pin;
        int _DIR_pin; // Pins du contrôleur de moteur
        int _PWM_pin;
        int encoder_pos = 0; // Position de l'encodeur
        int nb_rotation = 0;
        int encoder_max_pos = 800;
        int mean_time = 50; // Temps entre deux lectures de l'encodeur pour calculer la vitesse moyenne
        float set_speed = 0;
        float pulse_per_seconds_to_unit = 0.5; // Ratio entre le nombre de pulses par seconde et le ratio de vitesse
};

#endif //motor