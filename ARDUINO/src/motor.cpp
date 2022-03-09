/*
Projet S4 GRO 2022
Class to control Pololu 50:1 Metal Gearmotor 37Dx70L mm 12V 
with 64 CPR Encoder and with Cytron 13Amp 6V-30V DC Motor Driver 
on Arduino Mega 2560
@author Antoine Waltz et al
@version 1.0 09/02/2022
*/

#include "rufus_lib/motor.h"

motor::motor(){
    /*
    Constructeur par défaut qui assigne une valeur aux pins
    */
    _ENC_A_pin = -1;
    _ENC_B_pin = -1;
    _DIR_pin = -1;
    _PWM_pin = -1;
}
motor::motor(int ENC_A_pin, int ENC_B_pin, int DIR_pin, int PWM_pin){
    /*
    Constructeur qui vérifie la validité des pins entrées en paramètres et qui les assigne à ses attributs
    */
    if (motor::checkPins(ENC_A_pin, 1)){_ENC_A_pin = ENC_A_pin;}
    else{_ENC_A_pin = -1;}
    if (motor::checkPins(ENC_B_pin, 2)){_ENC_B_pin = ENC_B_pin;}
    else{_ENC_B_pin = -1;}
    if (motor::checkPins(DIR_pin, 3)){_DIR_pin = DIR_pin;}
    else{_DIR_pin = -1;}
    if (motor::checkPins(PWM_pin, 4)){_PWM_pin = PWM_pin;}
    else{_PWM_pin = -1;}
    motor::init();
}
motor::~motor(){
    /*
    Destructeur par défaut
    */
}
void motor::init(){
    /*
    Fonction qui initialise les pins sur l'Arduino
    */
    pinMode(motor::getPin(_ENC_A),INPUT);
    pinMode(motor::getPin(_ENC_B),INPUT);
    pinMode(motor::getPin(_DIR),OUTPUT);
    pinMode(motor::getPin(_PWM),OUTPUT);
}
bool motor::checkPins(int pin, int arg_nb){
    /*
    Fonction qui prend en paramètre une pin et son type selon 4 cas et qui vérifie si la pin fait partie des options disponible, retourne un booléen
    */
    switch (arg_nb){
        case 1:
            if (pin == 2 || pin == 3 || pin == 18 || pin == 19){
                return true;
            }
            else{
                Serial.println("Invalid pin number for _ENC_A_pin, choose either 2,3,18 or 19");
                return false;
            }
        case 2:
            if (pin >= 22 && pin <= 53){
                return true;
            }
            else{
                Serial.println("Invalid pin number for _ENC_B_pin, choose a pin between 22 and 53");
                return false;
            }
        case 3:
            if ((pin >= 22 && pin <= 53) || (pin >= 2 && pin <= 9) ){
                return true;
            }
            else{
                Serial.println("Invalid pin number for _DIR_pin, choose a pin between 22 and 53");
                return false;
            }
        case 4:
            if (pin >= 4 && pin <= 13){
            return true;
            }
            else{
                Serial.println("Invalid pin number for _PWM_pin, choose a pin between 4 and 13");
                return false;
            }
        default:
            Serial.println("Invalid pin number argument, choose between 1 and 4");
            return false;
        }
}
bool motor::safeCheck(){
    /*
    Fonction qui permet de vérifier si les pins ont bel et bien été assignées
    */
    if (motor::getPin(_ENC_A) == -1 || motor::getPin(_ENC_B) == -1 || motor::getPin(_DIR)== -1 || motor::getPin(_PWM) == -1) {return false;}
    else {return true;}
}
int motor::getPin(char _pinName[11]){
    /*
    Fonction qui retourne la valeur de la pin demandée
    */
    if (_pinName == _ENC_A){
        return _ENC_A_pin;
    }
    else if (_pinName == _ENC_B){
        return _ENC_B_pin;
    }
    else if (_pinName == _DIR){
        return _DIR_pin;
    }
    else if (_pinName == _PWM){
        return _PWM_pin;
    }
    else{
        Serial.println("Invalid pin name, choose either _ENC_A_pin, _ENC_B_pin, _DIR_pin or _PWM_pin");
        return -1;
    }
}
void motor::getPins(){
    /*
    Fonction qui affiche la valeur de toutes les pins
    */
    Serial.print(_ENC_A);
    Serial.print(" : ");
    Serial.println(_ENC_A_pin);
    Serial.print(_ENC_B);
    Serial.print(" : ");
    Serial.println(_ENC_B_pin);
    Serial.print(_DIR);
    Serial.print(" : ");
    Serial.println(_DIR_pin);
    Serial.print(_PWM);
    Serial.print(" : ");
    Serial.println(_PWM_pin);
}
void motor::readEncoder(){
    /*
    Fonction qui enregistre la position de l'encodeur sur un tour
    */
    if (digitalRead(_ENC_B_pin) > 0){
        encoder_pos++;
        }
    else {
        encoder_pos--;
        }
    if (encoder_pos >= encoder_max_pos){
        nb_rotation++;
        encoder_pos = encoder_pos - encoder_max_pos;
    }
    else if (encoder_pos <= -encoder_max_pos){
        nb_rotation--;
        encoder_pos = encoder_pos + encoder_max_pos;
    }
}
void motor::resetEncoder(){
    /*
    Fonction qui remet à zéro la position de l'encodeur
    */
    encoder_pos = 0;
    nb_rotation = 0;
}
int motor::getEncoderPos(){
    /*
    Fonction qui retourne la position de l'encodeur
    */
    return encoder_pos;
}
int motor::getNbRotation(){
    /*
    Fonction qui retourne le nombre de tours complets
    */
    return nb_rotation;
}
int motor::getEncoderPosTotal(){
    /*
    Fonction qui retourne la position absolue de l'encodeur depuis la dernière mise à zéro
    */
    return nb_rotation*encoder_max_pos + encoder_pos;
}
int motor::getResetEncoderPos(){
    /*
    Fonction qui retourne la position de l'encodeur sur un tour et qui réinitialise l'encodeur
    */
    int pos = motor::getEncoderPosTotal();
    motor::resetEncoder();
    return pos;
}
void motor::setPWM(float speed){
    /*
    Fonction qui permet de faire tourner le moteur selon un ratio de vitesse entre -1 et 1
    */
    set_speed = speed;
    int pwm = abs(speed*255.0f); // Change speed to unsigned integer
    // Switching polarity on DIR_PIN to change motor direction
    if (speed > 0) {
        digitalWrite(_DIR_pin, LOW);
    } else {
        digitalWrite(_DIR_pin, HIGH);
    }
    // SetPWM value
    analogWrite(_PWM_pin, (pwm > 255) ? 255 : static_cast<uint8_t>(pwm));
}
double motor::getMeanSpeed(){
    /*
    Fonction qui calcule et retourne la vitesse moyenne du moteur sur un intervalle de temps
    */
    int mean_time = 10; // ms, Temps entre deux lectures de l'encodeur pour calculer la vitesse moyenne
    int pos1 = motor::getEncoderPosTotal();
    delay(mean_time);
    int pos2 = motor::getEncoderPosTotal();
    return (double)((pos2-pos1))/(((mean_time))/1000.0)*pulse_per_seconds_to_unit_speed;
}