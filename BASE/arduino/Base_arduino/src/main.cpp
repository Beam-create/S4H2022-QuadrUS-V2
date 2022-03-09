/* 
 * GRO 400 - Conception agile et ouverte en robotique
 * Code de démarrage
 * Auteurs: Antoine Waltz et al     
 * date: 22 février 2022
*/

/*------------------------------ Librairies ---------------------------------*/

#include <Arduino.h>
#include "motor.h"
#include "pid.h"
#include "Servo.h"
#include <ros.h>
#include <base_control/Rufus_base_msgs.h>
#include <std_msgs/String.h>
#include <base_control/Feedback_arduino_msgs.h>

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

// ROS
ros::NodeHandle nh;
base_control::Feedback_arduino_msgs feedback_msg;

/*------------------------- Prototypes de fonctions -------------------------*/

void readEncoderFL();
void readEncoderFR();
void readEncoderBL();
void readEncoderBR();
void motorEncoderTest(int motorID);
void motorSpeedTest(int motorID, float speed);
void gainsTest(float cmd_FL, float cmd_FR, float cmd_BL, float cmd_BR);
void pidTest(float cmd_FL, float cmd_FR, float cmd_BL, float cmd_BR);
void moveForward(float speed);
void moveBackward(float speed);
void moveLeft(float speed);
void moveRight(float speed);
void moveDiagFL(float speed);
void moveDiagFR(float speed);
void moveDiagBL(float speed);
void moveDiagBR(float speed);
void rotate(float speed, int direction, int point_of_rotation);
void stop();
void demo();
void commandCB(const base_control::Rufus_base_msgs& motor_cmd);

/*---------------------------- fonctions "Main" -----------------------------*/

ros::Subscriber<base_control::Rufus_base_msgs> motor_sub("/rufus/base_arduino", commandCB);
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
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(arduino_feedback);
  nh.subscribe(motor_sub);
  nh.negotiateTopics();
}

void loop() {
  // demo();
  // arduino_feedback.publish(&feedback_msg);
  // nh.spinOnce();
  // delay(10);
  //gainsTest(0.5, 0.5, 0.5, 0.5);
  pidTest(0, 0, 0, 0);
  // BR_motor.setPWM(0.3);
  // motorSpeedTest(1,0.08);
  // motorEncoderTest(1);
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
// Fonction de test des encodeurs
void motorEncoderTest(int motorID){
  motor *moteur;
  if (motorID == 1){moteur = &FL_motor;}
  else if (motorID == 2){moteur = &FR_motor;}
  else if (motorID == 3){moteur = &BL_motor;}
  else if (motorID == 4){moteur = &BR_motor;}
  Serial.println("---------------------------");
  Serial.println("Motor Encoder Test :");
  Serial.println("---------------------------");
  if (!moteur->safeCheck()) {
    Serial.println("Destruction du moteur");
    BL_motor.~motor();
    }
  Serial.print("Nombre de tour : ");
  Serial.println(moteur->getNbRotation());
  Serial.print("Position de l'encodeur : ");
  Serial.println(moteur->getEncoderPos());
  Serial.print("Position de totale de l'encodeur : ");
  Serial.println(moteur->getEncoderPosTotal());
  Serial.print("Vitesse moyenne : ");
  Serial.println(moteur->getMeanSpeed());
  Serial.println("---------------------------");
  Serial.println("End of motor encoder test");
  Serial.println("---------------------------");
  Serial.println("");
  delay(500);
}
// Fonction de test du calcul de la vitesse moyenne d'un moteur
void motorSpeedTest(int motorID, float speed){
  motor *moteur;
  if (motorID == 1){moteur = &FL_motor;}
  else if (motorID == 2){moteur = &FR_motor;}
  else if (motorID == 3){moteur = &BL_motor;}
  else if (motorID == 4){moteur = &BR_motor;}
  Serial.println("---------------------------");
  Serial.println("Motor Speed Test :");
  Serial.println("---------------------------");
  Serial.print("Vitesse input : ");
  Serial.println(speed);
  moteur->setPWM(speed);
  // FL_motor.setPWM(speed);
  if (!moteur->safeCheck()) {
  // if (!FL_motor.safeCheck()) {
  Serial.println("Destruction du moteur");
  // moteur->~motor();
  FL_motor.~motor();
  }
  else{
  Serial.print("Vitesse moyenne : ");
  Serial.println(moteur->getMeanSpeed());
  // Serial.println(FL_motor.getMeanSpeed());
  }
  Serial.println("---------------------------");
  Serial.println("End of Speed Test");
  Serial.println("---------------------------");
  Serial.println("");
  delay(500);
}
// Fonction de test du PID
void pidTest(float cmd_FL, float cmd_FR, float cmd_BL, float cmd_BR){
  bool atteint = false;
  if(!atteint){
    Serial.println("---------------------------");
    Serial.println("PID Test :");
    Serial.println("---------------------------");
    Serial.print("Goal FL : ");
    Serial.println(cmd_FL);
    Serial.print("Goal FR : ");
    Serial.println(cmd_FR);
    Serial.print("Goal BL : ");
    Serial.println(cmd_BL);
    Serial.print("Goal BR : ");
    Serial.println(cmd_BR);
    Serial.println("---------------------------");
    pid.goals();
    // Serial.println(pid.goal());
    pid.run(cmd_FL, cmd_FR, cmd_BL, cmd_BR);
    if (!pid.goal()){
      Serial.println("---------------------------");
      Serial.println("Not at Goal");
      Serial.println("---------------------------");
      Serial.print("Vitesse moyenne FL : ");
      Serial.println(FL_motor.getMeanSpeed());
      Serial.print("Vitesse moyenne FR : ");
      Serial.println(FR_motor.getMeanSpeed());
      Serial.print("Vitesse moyenne BL : ");
      Serial.println(BL_motor.getMeanSpeed());
      Serial.print("Vitesse moyenne BR : ");
      Serial.println(BR_motor.getMeanSpeed());
    }
    if (pid.goal() && !atteint){
      Serial.println("---------------------------");
      Serial.println("Goal Reached");
      Serial.println("---------------------------");
      Serial.print("Vitesse moyenne FL : ");
      Serial.println(FL_motor.getMeanSpeed());
      Serial.print("Vitesse moyenne FR : ");
      Serial.println(FR_motor.getMeanSpeed());
      Serial.print("Vitesse moyenne BL : ");
      Serial.println(BL_motor.getMeanSpeed());
      Serial.print("Vitesse moyenne BR : ");
      Serial.println(BR_motor.getMeanSpeed());
      atteint = true;
    }
    if (atteint){
    Serial.println("---------------------------");
    Serial.println("End of PID test");
    Serial.println("---------------------------");
    Serial.println("");
    }
  }
}
void gainsTest(float cmd_FL, float cmd_FR, float cmd_BL, float cmd_BR){
  pid.run(cmd_FL, cmd_FR, cmd_BL, cmd_BR);
  // Serial.println(FL_motor.getMeanSpeed());
  Serial.println(FR_motor.getMeanSpeed());
  // Serial.println(BL_motor.getMeanSpeed());
  // Serial.println(BR_motor.getMeanSpeed());
}
// Fonctions pour la démo
void moveForward(float speed){
  FL_motor.setPWM(speed);
  FR_motor.setPWM(speed*-1);
  BL_motor.setPWM(speed);
  BR_motor.setPWM(speed*-1);
}
void moveBackward(float speed){
  FL_motor.setPWM(-speed);
  FR_motor.setPWM(-speed*-1);
  BL_motor.setPWM(-speed);
  BR_motor.setPWM(-speed*-1);
}
void moveLeft(float speed){
  FL_motor.setPWM(-speed);
  FR_motor.setPWM(speed*-1);
  BL_motor.setPWM(speed);
  BR_motor.setPWM(-speed*-1);
}
void moveRight(float speed){
  FL_motor.setPWM(speed);
  FR_motor.setPWM(-speed*-1);
  BL_motor.setPWM(-speed);
  BR_motor.setPWM(speed*-1);
}
void moveDiagFL(float speed){
  FL_motor.setPWM(0);
  FR_motor.setPWM(speed*-1);
  BL_motor.setPWM(speed);
  BR_motor.setPWM(0*-1);
}
void moveDiagFR(float speed){
  FL_motor.setPWM(speed);
  FR_motor.setPWM(0*-1);
  BL_motor.setPWM(0);
  BR_motor.setPWM(speed*-1);
}
void moveDiagBR(float speed){
  FL_motor.setPWM(0);
  FR_motor.setPWM(-speed*-1);
  BL_motor.setPWM(-speed);
  BR_motor.setPWM(0*-1);
}
void moveDiagBL(float speed){
  FL_motor.setPWM(-speed);
  FR_motor.setPWM(0*-1);
  BL_motor.setPWM(0);
  BR_motor.setPWM(-speed*-1);
}
void stop(){
  FL_motor.setPWM(0);
  FR_motor.setPWM(0*-1);
  BL_motor.setPWM(0);
  BR_motor.setPWM(0*-1);
}
void rotate(float speed, int direction, int point_of_rotation){
  float dir_speed = speed;
  Serial.println(dir_speed);
  if (direction < 0){
    dir_speed *= -1;
  }
  if (point_of_rotation == 0){
    FL_motor.setPWM(dir_speed);
    FR_motor.setPWM(-dir_speed*-1);
    BL_motor.setPWM(dir_speed);
    BR_motor.setPWM(-dir_speed*-1);
  }
  else if (point_of_rotation == 1){
    FL_motor.setPWM(dir_speed);
    FR_motor.setPWM(-dir_speed*-1);
    BL_motor.setPWM(0);
    BR_motor.setPWM(0*-1);
  }
  else if (point_of_rotation == 2){
    FL_motor.setPWM(0);
    FR_motor.setPWM(0*-1);
    BL_motor.setPWM(dir_speed);
    BR_motor.setPWM(-dir_speed*-1);
  }
  else if (point_of_rotation == 3){
    FL_motor.setPWM(dir_speed);
    FR_motor.setPWM(0*-1);
    BL_motor.setPWM(dir_speed);
    BR_motor.setPWM(0*-1);
  }
  else if (point_of_rotation == 4){
    FL_motor.setPWM(0);
    FR_motor.setPWM(dir_speed*-1);
    BL_motor.setPWM(0);
    BR_motor.setPWM(dir_speed*-1);
  }
}

void demo(){
  float speed = 0.5;
  int stop_time = 50;
  int move_time = 1000;

  // FL Square Loop
  moveForward(speed);
  delay(move_time);
  stop();
  delay(stop_time);
  moveLeft(speed);
  delay(move_time);
  stop();
  delay(stop_time);
  moveBackward(speed);
  delay(move_time);
  stop();
  delay(stop_time);
  moveRight(speed);
  delay(move_time);
  stop();
  delay(stop_time);

  // BR Square Loop
  moveRight(speed);
  delay(move_time);
  stop();
  delay(stop_time);
  moveBackward(speed);
  delay(move_time);
  stop();
  delay(stop_time);
  moveLeft(speed);
  delay(move_time);
  stop();
  delay(stop_time);
  moveForward(speed);
  delay(move_time);
  stop();
  delay(stop_time);

  // FL Diag Loop
  moveDiagFL(speed);
  delay(move_time);
  stop();
  delay(stop_time);
  moveDiagBL(speed);
  delay(move_time);
  stop();
  delay(stop_time);
  moveDiagBR(speed);
  delay(move_time);
  stop();
  delay(stop_time);
  moveDiagFR(speed);
  delay(move_time);
  stop();
  delay(stop_time);

  // FR Diag Loop
  moveDiagFR(speed);
  delay(move_time);
  stop();
  delay(stop_time);
  moveDiagBR(speed);
  delay(move_time);
  stop();
  delay(stop_time);
  moveDiagBL(speed);
  delay(move_time);
  stop();
  delay(stop_time);
  moveDiagFL(speed);
  delay(move_time);
  stop();
  delay(stop_time);

  // Clockwise, center
  rotate(speed,1,0);
  delay(move_time);
  stop();
  delay(stop_time);

  // CouterClockwise, center
  rotate(speed,-1,0);
  delay(move_time);
  stop();
  delay(stop_time);

  // Clockwise, back
  rotate(speed,1,1);
  delay(move_time);
  stop();
  delay(stop_time);

  // ?
  rotate(speed,-1,4);
  delay(move_time);
  stop();
  delay(stop_time);

}
void commandCB(const base_control::Rufus_base_msgs& motor_cmd)
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
void moveSequence(){
 // Attends de recevoir la confirmation de la part de la caméra que la balle se trouve dans son champs de vision
 // Lorsque la balle est aperçue, envoie une info qui détermine la direction de la rotation (càd selon si la balle est à gauche ou à droite du centre de l'image)
 // rotate(speed,direction,0);
 // Lorsque reçoit la confirmation que la balle est centrée sur l'image (donc que la base est alignée avec la balle)
 // stop();
 // moveForward(speed);
 // Lorsque reçoit la confirmation que la balle est à la portée du bras (portée absolue ou distance de positionnement désirée)
 // stop();
}