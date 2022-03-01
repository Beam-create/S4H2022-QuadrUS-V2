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
    interpQ1.go(GOAL[0]-currAngles[0], 2000, LINEAR, ONCEFORWARD);
    interpQ1.go(GOAL[1]-currAngles[1], 2000, LINEAR, ONCEFORWARD);
    interpQ1.go(GOAL[2]-currAngles[2], 2000, LINEAR, ONCEFORWARD);

    while(!interpQ1.isFinished() || !interpQ2.isFinished() || !interpQ3.isFinished()){
        float q1 = interpQ1.update() + currAngles[0];
        float q2 = interpQ2.update() + currAngles[1];
        float q3 = interpQ3.update() + currAngles[2];
        
        if((L1 + L2*sin(angles[1]*(PI/180.0)) - L3*sin(angles[2]*(PI/180.0)) - L4y)>=0.02){ // Verification de l'eq en y 
            Serial.print("Servo q1 commands are: ");
            Serial.println(q1);
            //Joints[i].writeMicroseconds(writeServo(i,GOAL[i]));
            Serial.print("Servo q2 commands are: ");
            Serial.println(q2);
            //Joints[i].writeMicroseconds(writeServo(i,GOAL[i]));
            Serial.print("Servo q3 commands are: ");
            Serial.println(q3);
            Serial.println("NEXT INTERPOLATION");
        }
        else{
            Serial.print("La combinaison d'ang en entree donne une valeur de: ");
            Serial.println(L1 + L2*sin(angles[1]*(PI/180.0)) + L3*sin(angles[2]*(PI/180.0)) - L4y);
        }
    }
}

void Bras_servo_control::goToHome()
{ 
    for (size_t i =0; i< nbJoints; i++){
        Joints[i].writeMicroseconds(writeServo(i,GOAL[i]));
        // Serial.print("Servo angle commands are: ");
        // Serial.println(GOAL[i]);
        currAngles[i] = GOAL[i];
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

// float Interpolation::go(float input, int duration)
// {
//     if (input != savedValue) {   // check for new data
//         interpolationFlag = 0;
//     }
//     savedValue = input;          // bookmark the old value  
    
//     if (interpolationFlag == 0) {                                        // only do it once until the flag is reset
//         myRamp.go(input, duration, LINEAR, ONCEFORWARD);              // start interpolation (value to go to, duration)
//         interpolationFlag = 1;
//     }
    
//     float output = myRamp.update();               
//     return output;
// }
