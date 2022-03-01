#include "Bras_servo_control.h"

Bras_servo_control bras = Bras_servo_control();
bool isDone = false; // Flag condition, testing only, if needed!

float JointAngles[nbJoints]= {26.57, 77.02, 34.87}; // Target angles are defined once;

void setup() {
  //Init Serial USB
  Serial.begin(9600);
  Serial.println(F("Initialize System"));
  bras.initServos();
  bras.drop();
  bras.goTo(bras.HOME);
  delay(3000);
}

void loop() {
    bras.goTo(JointAngles);
    delay(1000);
    bras.pick();
}