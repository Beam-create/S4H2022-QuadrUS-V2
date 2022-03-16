/* 
 * GRO 400 - Conception agile et ouverte en robotique
 * Code de démarrage
 * Auteurs: Antoine Waltz et al     
 * date: 22 février 2022
*/

/*------------------------------ Librairies ---------------------------------*/

#include <Arduino.h>
#include "rufus_lib/motor.h"
#include "rufus_lib/pid.h"
#include "rufus_lib/Bras_servo_control.h"
#include "Servo.h"
#include <ros.h>
#include <rufus_master/Rufus_base_msgs.h>
#include <rufus_master/bras_commands.h> // A generer!, voir Mat
#include <std_msgs/String.h>
#include <rufus_master/Feedback_arduino_msgs.h>

/*------------------------------ Constantes ---------------------------------*/

#define BAUD            9600      // Fréquence de transmission serielle

/*---------------------------- variables globales ---------------------------*/
 
 // ENC_A: Fil jaune de l'encodeur, soit 2,3,18 ou 19
int FL_ENC_A_pin = 19;
int FR_ENC_A_pin = 18;
int BL_ENC_A_pin = 2;
int BR_ENC_A_pin = 3;

// ENC_B: Fil blanc de l'encodeur, entre 22 et 53 (30,32,34,36)
int FL_ENC_B_pin = 30;
int FR_ENC_B_pin = 32;
int BL_ENC_B_pin = 36;
int BR_ENC_B_pin = 34;

// ENC_Vcc: Fil bleu de l'encodeur, 3.5-20 V
// ENC_GND: Fil vert de l'encodeur

// DIR: Fil jaune du contrôleur de moteur, entre 22 et 53 (22,24,26,28)
int FL_DIR_pin = 22;
int FR_DIR_pin = 24;
int BL_DIR_pin = 26;
int BR_DIR_pin = 28;

// PWM: Fil blanc du contrôleur de moteur, entre 4 et 13 (4,5,6,7)
int FL_PWM_pin = 4;
int FR_PWM_pin = 5;
int BL_PWM_pin = 6;
int BR_PWM_pin = 7;

// Pin gimbal
int gimbal_pin = 8;


// Déclaration des objets moteur
motor FL_motor(FL_ENC_A_pin, FL_ENC_B_pin, FL_DIR_pin, FL_PWM_pin);
motor *FL = &FL_motor;
motor FR_motor(FR_ENC_A_pin, FR_ENC_B_pin, FR_DIR_pin, FR_PWM_pin);
motor *FR = &FR_motor;
motor BL_motor(BL_ENC_A_pin, BL_ENC_B_pin, BL_DIR_pin, BL_PWM_pin);
motor *BL = &BL_motor;
motor BR_motor(BR_ENC_A_pin, BR_ENC_B_pin, BR_DIR_pin, BR_PWM_pin);
motor *BR = &BR_motor;

// Déclaration de l'objet PID
PID pid(*FL, *FR, *BL, *BR);

// Déclaration de l'objet Gimbal
Servo gimbal;

// Declaration de l'objet Bras et autre variables
Bras_servo_control bras;
const long interpolTime = 7;
unsigned long currentTime;

// ROS
ros::NodeHandle nh;
rufus_master::Feedback_arduino_msgs feedback_msg;

/*------------------------- Prototypes de fonctions -------------------------*/

void readEncoderFL();
void readEncoderFR();
void readEncoderBL();
void readEncoderBR();
void commandCB(const rufus_master::Rufus_base_msgs& motor_cmd);
void brasCB(const rufus_master::bras_commands& bras_cmd);


/*---------------------------- fonctions "Main" -----------------------------*/

ros::Subscriber<rufus_master::Rufus_base_msgs> motor_sub("/rufus/base_arduino", commandCB);
ros::Subscriber<rufus_master::bras_commands> bras_sub("/rufus/bras_arduino", brasCB); // Get les commands via le topic envoyer par bras_teleop et IK
ros::Publisher arduino_feedback("/rufus/arduino_feedback",&feedback_msg);

void setup() {
  Serial.begin(BAUD);
  // Fonctions qui assigne les pins à lire et les fonctions pour lire les encodeurs
  // attachInterrupt: 2, 3, 18, 19, 20, 21 (pins 20 & 21 are not available to use for interrupts while they are used for I2C communication)
  attachInterrupt(digitalPinToInterrupt(FL_motor.getPin(FL_motor._ENC_A)), readEncoderFL, RISING);
  attachInterrupt(digitalPinToInterrupt(FR_motor.getPin(FR_motor._ENC_A)), readEncoderFR, RISING);
  attachInterrupt(digitalPinToInterrupt(BL_motor.getPin(BL_motor._ENC_A)), readEncoderBL, RISING);
  attachInterrupt(digitalPinToInterrupt(BR_motor.getPin(BR_motor._ENC_A)), readEncoderBR, RISING);
  pid.setGains(1, 0.1, 0.05);
  gimbal.attach(gimbal_pin);
  gimbal.write(90); // Angle zéro à trouver, en degré

  bras.drop();
  bras.goToHome();

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(arduino_feedback);
  nh.subscribe(motor_sub);
  nh.negotiateTopics();
}

void loop() {
  // arduino_feedback.publish(&feedback_msg);
  // nh.spinOnce();
  // delay(10);
}

/*---------------------------Definition de fonctions ------------------------*/
// Fonctions qui appelle la fonction de lecture de la position de l'encodeur
void readEncoderFL(){
  FL_motor.readEncoder();
}
void readEncoderFR(){
  FR_motor.readEncoder();
}
void readEncoderBL(){
  BL_motor.readEncoder();
}
void readEncoderBR(){
  BR_motor.readEncoder();
}

void commandCB(const rufus_master::Rufus_base_msgs& motor_cmd)
{
  float cmd_FL = motor_cmd.motor_FL;
  float cmd_FR = motor_cmd.motor_FR;
  float cmd_BL = motor_cmd.motor_BL;
  float cmd_BR = motor_cmd.motor_BR;

  FL_motor.setPWM(cmd_FL);
  FR_motor.setPWM(cmd_FR);
  BL_motor.setPWM(cmd_BL);
  BR_motor.setPWM(cmd_BR);
  
  //Ajouter les fonctions pour faire tourner les moteurs en fonction des cmd_XX
  // pid.run(cmd_FL, cmd_FR, cmd_BL, cmd_BR);

  feedback_msg.motor_FL = cmd_FL;
  feedback_msg.motor_FR = cmd_FR;
  feedback_msg.motor_BL = cmd_BL;
  feedback_msg.motor_BR = cmd_BR;
  // feedback_msg.mean_speed_FL = pid._FL_motor->getMeanSpeed();
  // feedback_msg.mean_speed_FR = pid._FR_motor->getMeanSpeed();
  // feedback_msg.mean_speed_BL = pid._BL_motor->getMeanSpeed();
  // feedback_msg.mean_speed_BR = pid._BR_motor->getMeanSpeed();
  // feedback_msg.encoder_FL = pid._FL_motor->getEncoderPos();
  // feedback_msg.encoder_FR = pid._FR_motor->getEncoderPos();
  // feedback_msg.encoder_BL = pid._BL_motor->getEncoderPos();
  // feedback_msg.encoder_BR = pid._BR_motor->getEncoderPos();

}

void brasCB(const rufus_master::bras_commands& bras_cmd){
  // Tsansfert des angles dans des variables locals
  float angles[nbJoints];
  angles[0] = bras_cmd.q1;
  angles[1] = bras_cmd.q2;
  angles[2] = bras_cmd.q3;

  // Executes Automatic mode
  if(bras_cmd.FK){
    bool isDone = false;
    float smoothAngles[nbJoints];

    while(!isDone){
      for (int i=0; i<nbJoints; i++)
      {
        smoothAngles[i] = (angles[i]*0.03) + (bras.prevAngles[i]*0.97);
        bras.prevAngles[i] = smoothAngles[i];
      }
      bras.goTo(smoothAngles);
      delay(7);
      if((round(smoothAngles[0]*100)/100.0) == angles[0] && (round(smoothAngles[1]*100)/100.0) == angles[1] && (round(smoothAngles[2]*100)/100.0) == angles[2]) // Sync once all 3 values have reached
      {
        isDone = true;
        bras.setPrevAngles(smoothAngles);
        // Serial.println("GOAL has been reached!"); //debugging
      }
    }
  }
  // Mode manuel
  else{
    bras.goTo(angles);
  }

  // Check le state de leffecteur
  if(bras.isPick){
    feedback_msg.effector = bras.isPick;
  }
  else{
    feedback_msg.effector = false;
  }

  feedback_msg.q1 = angles[0];
  feedback_msg.q2 = angles[1];
  feedback_msg.q3 = angles[2];
}

