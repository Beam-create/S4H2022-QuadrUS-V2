#include "Bras_servo_control.h"

Bras_servo_control bras = Bras_servo_control();
bool isDone = false; // Flag condition, testing only, if needed!

float JointAngles[nbJoints]= {0.00, 80.00, 40.00}; // Target angles are defined once;
float smoothAngles[nbJoints];

void setup() {
  //Init Serial USB
  Serial.begin(9600);
  Serial.println(F("Initialize System"));
  bras.initServos();
  bras.drop();
  bras.goToHome();
  delay(3000);
}

void loop() {
  while(!isDone){
    for (int i=0; i<nbJoints; i++){
      smoothAngles[i] = (JointAngles[i]*0.03) + (bras.prevAngles[i]*0.97);
      bras.prevAngles[i] = smoothAngles[i];
    }
    bras.goTo(smoothAngles);
    delay(7);
    //Serial.println(smoothAngles[1]);
    
    if((round(smoothAngles[0]*100)/100.0) == JointAngles[0] && (round(smoothAngles[1]*100)/100.0) == JointAngles[1] && (round(smoothAngles[2]*100)/100.0) == JointAngles[2]) // Sync once all 3 values have reached
    {
      isDone = true;
      bras.setPrevAngles(smoothAngles);
      Serial.println("GOAL has been reached!");
   }
  }
  delay(100);
  bras.pick();
  // delay(2000);
  // bras.goToHome();
}
