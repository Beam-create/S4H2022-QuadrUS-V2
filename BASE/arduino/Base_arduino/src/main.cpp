/* 
 * GRO 400 - Conception agile et ouverte en robotique
 * Code de démarrage
 * Auteurs: Antoine Waltz et al     
 * date: 9 février 2022
*/

/*------------------------------ Librairies ---------------------------------*/

#include <Arduino.h>
#include "motor.h"
#include "pid.h"

/*------------------------------ Constantes ---------------------------------*/

#define BAUD            9600      // Frequence de transmission serielle
#define NB_PULSE_TOUR   800

/*---------------------------- variables globales ---------------------------*/
 
 // ENC_A: Yellow wire out of encoder, either 2,3,18 or 19
int FL_ENC_A_pin = 19;
int FR_ENC_A_pin = 18;
int BL_ENC_A_pin = 2;
int BR_ENC_A_pin = 3;

// ENC_B: White wire out of encoder, between 22 and 53 (30,32,34,36)
int FL_ENC_B_pin = 30;
int FR_ENC_B_pin = 32;
int BL_ENC_B_pin = 36;
int BR_ENC_B_pin = 34;

// ENC_Vcc: Blue wire out of encoder, 3.5-20 V
// ENC_GND: Green wire out of encoder, any ground pin

// DIR: Yellow wire out of driver, between 22 and 53 (22,24,26,28)
int FL_DIR_pin = 22;
int FR_DIR_pin = 24;
int BL_DIR_pin = 26;
int BR_DIR_pin = 28;

// PWM: White wire out of driver, between 4 and 13 (4,5,6,7)
int FL_PWM_pin = 4;
int FR_PWM_pin = 5;
int BL_PWM_pin = 6;
int BR_PWM_pin = 7;

motor FL_motor(FL_ENC_A_pin, FL_ENC_B_pin, FL_DIR_pin, FL_PWM_pin);
motor FR_motor(FR_ENC_A_pin, FR_ENC_B_pin, FR_DIR_pin, FR_PWM_pin);
motor BL_motor(BL_ENC_A_pin, BL_ENC_B_pin, BL_DIR_pin, BL_PWM_pin);
motor BR_motor(BR_ENC_A_pin, BR_ENC_B_pin, BR_DIR_pin, BR_PWM_pin);
PID pid(FL_motor, FR_motor, BL_motor, BR_motor);

/*------------------------- Prototypes de fonctions -------------------------*/

void readEncoderFL();
void readEncoderFR();
void readEncoderBL();
void readEncoderBR();
void motorEncoderTest();
void motorSpeedTest();
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

/*---------------------------- fonctions "Main" -----------------------------*/

void setup() {
  Serial.begin(BAUD);
  // attachInterrupt: 2, 3, 18, 19, 20, 21 (pins 20 & 21 are not available to use for interrupts while they are used for I2C communication)
  attachInterrupt(digitalPinToInterrupt(FL_motor.getPin(FL_motor._ENC_A)), readEncoderFL, RISING);
  attachInterrupt(digitalPinToInterrupt(FR_motor.getPin(FR_motor._ENC_A)), readEncoderFR, RISING);
  attachInterrupt(digitalPinToInterrupt(BL_motor.getPin(BL_motor._ENC_A)), readEncoderBL, RISING);
  attachInterrupt(digitalPinToInterrupt(BR_motor.getPin(BR_motor._ENC_A)), readEncoderBR, RISING);
}

void loop() {
  // motorEncoderTest();
  // motorSpeedTest();
  // BL_motor.setPWM(0);
  delay(5000);
  //rotate(0.5,1,0);
  demo();
}

/*---------------------------Definition de fonctions ------------------------*/

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
void motorEncoderTest(){
  Serial.println("---------------------------");
  Serial.println("Motor Encoder Test :");
  Serial.println("---------------------------");
  if (!BL_motor.safeCheck()) {
    Serial.println("Destruction du moteur");
    BL_motor.~motor();
    }
  Serial.print("Nombre de tour : ");
  Serial.println(BL_motor.getNbRotation());
  Serial.print("Position de l'encodeur : ");
  Serial.println(BL_motor.getEncoderPos());
  Serial.print("Position de totale de l'encodeur : ");
  Serial.println(BL_motor.getEncoderPosTotal());
  Serial.print("Vitesse moyenne : ");
  Serial.println(BL_motor.getMeanSpeed());
  Serial.println("---------------------------");
  Serial.println("End of motor encoder test");
  Serial.println("---------------------------");
  Serial.println("");
  delay(500);
}

void motorSpeedTest(){
  Serial.println("---------------------------");
  Serial.println("Motor Speed Test :");
  Serial.println("---------------------------");
  if (!FL_motor.safeCheck()) {
  Serial.println("Destruction du moteur");
  FL_motor.~motor();
  Serial.print("Vitesse moyenne : ");
  Serial.println(FL_motor.getMeanSpeed());
  Serial.print("Vitesse input : ");
  FL_motor.setPWM(0.1);
  }
}

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

  // // FR Square Loop
  // moveForward(speed);
  // delay(move_time);
  // stop();
  // delay(stop_time);
  // moveRight(speed);
  // delay(move_time);
  // stop();
  // delay(stop_time);
  // moveBackward(speed);
  // delay(move_time);
  // stop();
  // delay(stop_time);
  // moveLeft(speed);
  // delay(move_time);
  // stop();
  // delay(stop_time);

  // // BL Square Loop
  // moveLeft(speed);
  // delay(move_time);
  // stop();
  // delay(stop_time);
  // moveBackward(speed);
  // delay(move_time);
  // stop();
  // delay(stop_time);
  // moveRight(speed);
  // delay(move_time);
  // stop();
  // delay(stop_time);
  // moveForward(speed);
  // delay(move_time);
  // stop();
  // delay(stop_time);

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