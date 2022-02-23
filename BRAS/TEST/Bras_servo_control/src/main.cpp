#include "Bras_servo_control.h"

Bras_servo_control bras = Bras_servo_control();
bool isDone = false;

void setup() {
  //Init Serial USB
  Serial.begin(9600);
  Serial.println(F("Initialize System"));
  bras.initServos();
  bras.drop();
  delay(3000);
}

void loop() {
  while (!isDone){
    // put your main code here, to run repeatedly:
    float JointAngles[nbJoints]= {0.0, 91.0, -31.0};
    //bras.readAngles(JointAngles);
    bras.goTo(JointAngles);
    //bras.goToHome();

    bras.pick();
    isDone = true;
  }
}