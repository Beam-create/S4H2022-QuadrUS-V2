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

int ENC_A_pin = 19; // Yellow wire out of encoder, either 2,3,18 or 19
int ENC_B_pin = 30; // White wire out of encoder, between 22 and 53 (30,32,34,36)
// int ENC_VCC_pin = ; // Blue wire out of encoder, 3.5-20 V
// int ENC_GND_pin = ; // Green wire out of encoder, any ground pin
int DIR_pin = 28; // Yellow wire out of driver, between 22 and 53 (22,24,26,28)
int PWM_pin = 12; // White wire out of driver, between 4 and 13 (4,5,6,7)
motor moteur_AD(ENC_A_pin, ENC_B_pin, DIR_pin, PWM_pin);
motor moteur_AG(ENC_A_pin, ENC_B_pin, DIR_pin, PWM_pin);
motor moteur_RD(ENC_A_pin, ENC_B_pin, DIR_pin, PWM_pin);
motor moteur_RG(ENC_A_pin, ENC_B_pin, DIR_pin, PWM_pin);
PID pid(moteur_AD, moteur_AG, moteur_RD, moteur_RG);

/*------------------------- Prototypes de fonctions -------------------------*/

void readEncoder();
void motorEncoderTest();
void motorSpeedTest();

/*---------------------------- fonctions "Main" -----------------------------*/

void setup() {
  Serial.begin(BAUD);
  attachInterrupt(digitalPinToInterrupt(moteur_AD.getPin(moteur_AD._ENC_A)), readEncoder, RISING); // 2, 3, 18, 19, 20, 21 (pins 20 & 21 are not available to use for interrupts while they are used for I2C communication)
}

void loop() {
  motorEncoderTest();
  // motorSpeedTest();
  //moteur_AD.setPWM(0);
  delay(500);
}

/*---------------------------Definition de fonctions ------------------------*/

void readEncoder(){
  moteur_AD.readEncoder();
}
void motorEncoderTest(){
  Serial.println("---------------------------");
  Serial.println("Motor Encoder Test :");
  Serial.println("---------------------------");
  if (!moteur_AD.safeCheck()) {
    Serial.println("Destruction du moteur");
    moteur_AD.~motor();
    }
  Serial.print("Nombre de tour : ");
  Serial.println(moteur_AD.getNbRotation());
  Serial.print("Position de l'encodeur : ");
  Serial.println(moteur_AD.getEncoderPos());
  Serial.print("Position de totale de l'encodeur : ");
  Serial.println(moteur_AD.getEncoderPosTotal());
  Serial.print("Vitesse moyenne : ");
  Serial.println(moteur_AD.getMeanSpeed());
  Serial.println("---------------------------");
  Serial.println("End of motor encoder test");
  Serial.println("---------------------------");
  Serial.println("");
}

void motorSpeedTest(){
  Serial.println("---------------------------");
  Serial.println("Motor Speed Test :");
  Serial.println("---------------------------");
  if (!moteur_AD.safeCheck()) {
  Serial.println("Destruction du moteur");
  moteur_AD.~motor();
  Serial.print("Vitesse moyenne : ");
  Serial.println(moteur_AD.getMeanSpeed());
  Serial.print("Vitesse input : ");
  moteur_AD.setPWM(0.1);
  }
}