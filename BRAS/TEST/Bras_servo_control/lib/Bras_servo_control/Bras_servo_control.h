
/********
 * Fichier: Bras_servo_control.h
 * Auteurs: Jordan Simard - Christopher Pacheco
 * Date: 9 fevrier 2022
 * Description: Implementation des methodes des classes decrites dans Bras_servo_control.cpp.
********/

#include <Arduino.h>
#include <Wire.h>//https://www.arduino.cc/en/reference/wire
#include <Servo.h>

#define nbJoints 3 

// //Parameters



class Bras_servo_control
{
private:
    /* data */
    Servo Joints[nbJoints];
    Servo Effecteur;

    uint8_t J_pins[nbJoints] = {2, 3, 4};

    float MIN_ANG[nbJoints] = {0.0, 0.0, 0.0};
    float MAX_ANG[nbJoints] = {270.0, 270.0, 270.0};
    float HOME[nbJoints] = {0.0, 0.0, 0.0};
    float GOAL[nbJoints] = {0.0, 0.0, 0.0};

    int pickAngle = 100;
    int dropAngle = 0;

public:
    Bras_servo_control();
    ~Bras_servo_control();
    void initServos();
    void readAngles(float angles[nbJoints]);
    void goTo();
    void pick();
    void drop();
};


