/*
Projet S4 GRO 2022
Class to control Pololu 50:1 Metal Gearmotor 37Dx70L mm 12V 
with 64 CPR Encoder and with Cytron 13Amp 6V-30V DC Motor Driver 
on Arduino Mega 2560
@author Antoine Waltz et al
@version 1.0 09/02/2022
*/

#include "motor.h"

motor::motor(){
    _ENC_A_pin = -1;
    _ENC_B_pin = -1;
    _DIR_pin = -1;
    _PWM_pin = -1;
}
motor::motor(int ENC_A_pin, int ENC_B_pin, int DIR_pin, int PWM_pin){
    // motor::setPin(_ENC_A, ENC_A_pin);
    // motor::setPin(_ENC_B, ENC_B_pin);
    // motor::setPin(_DIR, DIR_pin);
    // motor::setPin(_PWM, PWM_pin);
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
    
}
void motor::init(){
    pinMode(motor::getPin(_ENC_A),INPUT);
    pinMode(motor::getPin(_ENC_B),INPUT);
    pinMode(motor::getPin(_DIR),OUTPUT);
    pinMode(motor::getPin(_PWM),OUTPUT);
    // pinMode(_ENC_A_pin,INPUT);
    // pinMode(_ENC_B_pin,INPUT);
    // pinMode(_DIR_pin,OUTPUT);
    // pinMode(_PWM_pin,OUTPUT);
}
bool motor::checkPins(int pin, int arg_nb){
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
    if (motor::getPin(_ENC_A) == -1 || motor::getPin(_ENC_B) == -1 || motor::getPin(_DIR)== -1 || motor::getPin(_PWM) == -1) {return false;}
    else {return true;}
}
// void motor::setPin(char _pinName[11], int pinValue){
//     if (_pinName == _ENC_A){
//         if (pinValue == 2 || pinValue == 3 || pinValue == 18 || pinValue == 19){
//             _ENC_A_pin = pinValue;
//         }
//         else{
//             Serial.println("Invalid pin number for _ENC_A_pin, choose either 2,3,18 or 19");
//         }
//     }
//     else if (_pinName == _ENC_B){
//         if (pinValue >= 22 && pinValue <= 53){
//             _ENC_B_pin = pinValue;
//         }
//         else{
//             Serial.println("Invalid pin number for _ENC_B_pin, choose a pin between 22 and 53");
//         }
//     }
//     else if (_pinName == _DIR){
//         if (pinValue >= 22 && pinValue <= 53){
//             _DIR_pin = pinValue;
//         }
//         else{
//             Serial.println("Invalid pin number for _DIR_pin, choose a pin between 22 and 53");
//         }
//     }
//     else if (_pinName == _PWM){
//         if (pinValue >= 4 && pinValue <= 13){
//             _DIR_pin = pinValue;
//         }
//         else{
//             Serial.println("Invalid pin number for _PWM_pin, choose a pin between 4 and 13");
//         }
//     }
//     else{
//         Serial.println("Invalid pin name, choose either _ENC_A_pin, _ENC_B_pin, _DIR_pin or _PWM_pin");
//     }
// }
int motor::getPin(char _pinName[11]){
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
    encoder_pos = 0;
    nb_rotation = 0;
}
int motor::getEncoderPos(){
    return encoder_pos;
}
int motor::getNbRotation(){
    return nb_rotation;
}
int motor::getEncoderPosTotal(){
    return nb_rotation*encoder_max_pos + encoder_pos;
}
int motor::getResetEncoderPos(){
    int pos = motor::getEncoderPosTotal();
    motor::resetEncoder();
    return pos;
}
void motor::setPWM(float speed){
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
    int pos1 = motor::getEncoderPosTotal();
    delay(mean_time);
    int pos2 = motor::getEncoderPosTotal();
    return ((pos2-pos1))/(mean_time/1000.0)*pulse_per_seconds_to_unit;
}