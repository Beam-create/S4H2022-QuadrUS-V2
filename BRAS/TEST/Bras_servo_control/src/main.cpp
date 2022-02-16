#include "Bras_servo_control.h"

Bras_servo_control bras = Bras_servo_control();
bool isDone = false;

void setup() {
  //Init Serial USB
  Serial.begin(9600);
  Serial.println(F("Initialize System"));
  bras.initServos();
}

void loop() {
  while (!isDone){
    // put your main code here, to run repeatedly:
    float JointAngles[nbJoints]= {90.0, 0.0, 0.0};
    bras.readAngles(JointAngles);
    bras.goTo();
    isDone = true;
  }
}