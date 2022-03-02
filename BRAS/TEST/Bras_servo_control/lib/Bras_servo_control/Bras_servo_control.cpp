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

/**
 * This method attaches all joints and effector to the respective pins
 * @param : void
 * @return : void 
 */ 
void Bras_servo_control::initServos()
{
    for (size_t i = 0; i < nbJoints; i++)
    {
        Joints[i].attach(J_pins[i]);
    }
    Effecteur.attach(Eff_pin);
}

/**
 * This method converts the input angles from the inverse kinematic formula and sets the appropriate PWM signal to the servos.
 * Array is sorted from J1, J2 to Jend
 * @param angles array of goal angles to go to 
 * @return void 
 */ 
void Bras_servo_control::goTo(float angles[nbJoints])
{
    GOAL = convToServAng(angles); 
    if((L1 + L2*sin(angles[1]*(PI/180.0)) - L3*sin(angles[2]*(PI/180.0)) - L4y)>=0.02){ // Verification de l'eq en y, pour pas causer de colisions
        for (size_t i=0; i<nbJoints; i++){
            Joints[i].writeMicroseconds(writeServo(i,GOAL[i]));
        }
    }
    else{
        Serial.print("La combinaison d'ang en entree donne une valeur de: ");
        Serial.println(L1 + L2*sin(angles[1]*(PI/180.0)) + L3*sin(angles[2]*(PI/180.0)) - L4y);
    }
}

void Bras_servo_control::goToHome()
{ 
    for (size_t i =0; i< nbJoints; i++){
        Joints[i].writeMicroseconds(writeServo(i,HOME[i]));
        prevAngles[i] = HOME[i];
    }
}

float* Bras_servo_control::convToServAng(float angles[nbJoints])
{
    float* conv;
    conv[0] = ZEROS[0]+(angles[0]*gearRatio);
    conv[1] = (ZEROS[1]+90)-angles[1];
    conv[2] = ZEROS[2] - angles[2];
    return conv;
}

void Bras_servo_control::pick()
{
    Effecteur.write(pickAngle);
}

void Bras_servo_control::drop()
{
    Effecteur.write(dropAngle);
}

int Bras_servo_control::writeServo(int indJoint, float angle)
{
    int Micro_sec = COEFFS[indJoint][0]*pow(angle,3) + COEFFS[indJoint][1]*pow(angle,2) + COEFFS[indJoint][2]*angle + COEFFS[indJoint][3];
    return Micro_sec;
}


