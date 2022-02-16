/********
 * Fichier: Bras_servo_control.cpp
 * Auteurs: Jordan Simard - Christopher Pacheco
 * Date: 9 fevrier 2022
 * Description: Implementation des methodes des classes decrites dans Bras_servo_control.cpp.
********/

//Libraries
#include "Bras_servo_control.h"


Bras_servo_control::Bras_servo_control()
{
    Effecteur = Servo();
    for (size_t i=0;i<nbJoints;i++){
        Joints[i] = Servo();
    }
}

Bras_servo_control::~Bras_servo_control()
{
}

void Bras_servo_control::initServos()
{
    for (size_t i = 0; i < nbJoints; i++)
    {
        Joints[i].attach(J_pins[i]);
    }
}

void Bras_servo_control::readAngles(float angles[nbJoints])
{
    for (size_t i = 0; i<nbJoints; i++){
        GOAL[i] = map(angles[i], MIN_ANG[i], MAX_ANG[i], 0.0, 200.0); //180 is the value that defines the max angle for Servo object
    }
}

void Bras_servo_control::goTo()
{
    for (size_t i =0; i< nbJoints; i++){
        Joints[i].write(GOAL[i]);
        Serial.print("PWM commands are: ");
        Serial.println(GOAL[i]);
    }
}

void Bras_servo_control::pick()
{
    Effecteur.write(pickAngle);
}

void Bras_servo_control::drop()
{
    Effecteur.write(dropAngle);
}



