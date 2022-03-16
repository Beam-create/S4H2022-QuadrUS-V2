
// // Fonction de test des encodeurs
// void motorEncoderTest(int motorID){
//   motor *moteur;
//   if (motorID == 1){moteur = &FL_motor;}
//   else if (motorID == 2){moteur = &FR_motor;}
//   else if (motorID == 3){moteur = &BL_motor;}
//   else if (motorID == 4){moteur = &BR_motor;}
//   Serial.println("---------------------------");
//   Serial.println("Motor Encoder Test :");
//   Serial.println("---------------------------");
//   if (!moteur->safeCheck()) {
//     Serial.println("Destruction du moteur");
//     BL_motor.~motor();
//     }
//   Serial.print("Nombre de tour : ");
//   Serial.println(moteur->getNbRotation());
//   Serial.print("Position de l'encodeur : ");
//   Serial.println(moteur->getEncoderPos());
//   Serial.print("Position de totale de l'encodeur : ");
//   Serial.println(moteur->getEncoderPosTotal());
//   Serial.print("Vitesse moyenne : ");
//   Serial.println(moteur->getMeanSpeed());
//   Serial.println("---------------------------");
//   Serial.println("End of motor encoder test");
//   Serial.println("---------------------------");
//   Serial.println("");
//   delay(500);
// }
// // Fonction de test du calcul de la vitesse moyenne d'un moteur
// void motorSpeedTest(int motorID, float speed){
//   motor *moteur;
//   if (motorID == 1){moteur = &FL_motor;}
//   else if (motorID == 2){moteur = &FR_motor;}
//   else if (motorID == 3){moteur = &BL_motor;}
//   else if (motorID == 4){moteur = &BR_motor;}
//   Serial.println("---------------------------");
//   Serial.println("Motor Speed Test :");
//   Serial.println("---------------------------");
//   Serial.print("Vitesse input : ");
//   Serial.println(speed);
//   moteur->setPWM(speed);
//   // FL_motor.setPWM(speed);
//   if (!moteur->safeCheck()) {
//   // if (!FL_motor.safeCheck()) {
//   Serial.println("Destruction du moteur");
//   // moteur->~motor();
//   FL_motor.~motor();
//   }
//   else{
//   Serial.print("Vitesse moyenne : ");
//   Serial.println(moteur->getMeanSpeed());
//   // Serial.println(FL_motor.getMeanSpeed());
//   }
//   Serial.println("---------------------------");
//   Serial.println("End of Speed Test");
//   Serial.println("---------------------------");
//   Serial.println("");
//   delay(500);
// }
// // Fonction de test du PID
// void pidTest(float cmd_FL, float cmd_FR, float cmd_BL, float cmd_BR){
//   bool atteint = false;
//   if(!atteint){
//     Serial.println("---------------------------");
//     Serial.println("PID Test :");
//     Serial.println("---------------------------");
//     Serial.print("Goal FL : ");
//     Serial.println(cmd_FL);
//     Serial.print("Goal FR : ");
//     Serial.println(cmd_FR);
//     Serial.print("Goal BL : ");
//     Serial.println(cmd_BL);
//     Serial.print("Goal BR : ");
//     Serial.println(cmd_BR);
//     Serial.println("---------------------------");
//     pid.goals();
//     // Serial.println(pid.goal());
//     pid.run(cmd_FL, cmd_FR, cmd_BL, cmd_BR);
//     if (!pid.goal()){
//       Serial.println("---------------------------");
//       Serial.println("Not at Goal");
//       Serial.println("---------------------------");
//       Serial.print("Vitesse moyenne FL : ");
//       Serial.println(FL_motor.getMeanSpeed());
//       Serial.print("Vitesse moyenne FR : ");
//       Serial.println(FR_motor.getMeanSpeed());
//       Serial.print("Vitesse moyenne BL : ");
//       Serial.println(BL_motor.getMeanSpeed());
//       Serial.print("Vitesse moyenne BR : ");
//       Serial.println(BR_motor.getMeanSpeed());
//     }
//     if (pid.goal() && !atteint){
//       Serial.println("---------------------------");
//       Serial.println("Goal Reached");
//       Serial.println("---------------------------");
//       Serial.print("Vitesse moyenne FL : ");
//       Serial.println(FL_motor.getMeanSpeed());
//       Serial.print("Vitesse moyenne FR : ");
//       Serial.println(FR_motor.getMeanSpeed());
//       Serial.print("Vitesse moyenne BL : ");
//       Serial.println(BL_motor.getMeanSpeed());
//       Serial.print("Vitesse moyenne BR : ");
//       Serial.println(BR_motor.getMeanSpeed());
//       atteint = true;
//     }
//     if (atteint){
//     Serial.println("---------------------------");
//     Serial.println("End of PID test");
//     Serial.println("---------------------------");
//     Serial.println("");
//     }
//   }
// }
// void gainsTest(float cmd_FL, float cmd_FR, float cmd_BL, float cmd_BR){
//   pid.run(cmd_FL, cmd_FR, cmd_BL, cmd_BR);
//   // Serial.println(FL_motor.getMeanSpeed());
//   Serial.println(FR_motor.getMeanSpeed());
//   // Serial.println(BL_motor.getMeanSpeed());
//   // Serial.println(BR_motor.getMeanSpeed());
// }
// // Fonctions pour la démo
// void moveForward(float speed){
//   FL_motor.setPWM(speed);
//   FR_motor.setPWM(speed*-1);
//   BL_motor.setPWM(speed);
//   BR_motor.setPWM(speed*-1);
// }
// void moveBackward(float speed){
//   FL_motor.setPWM(-speed);
//   FR_motor.setPWM(-speed*-1);
//   BL_motor.setPWM(-speed);
//   BR_motor.setPWM(-speed*-1);
// }
// void moveLeft(float speed){
//   FL_motor.setPWM(-speed);
//   FR_motor.setPWM(speed*-1);
//   BL_motor.setPWM(speed);
//   BR_motor.setPWM(-speed*-1);
// }
// void moveRight(float speed){
//   FL_motor.setPWM(speed);
//   FR_motor.setPWM(-speed*-1);
//   BL_motor.setPWM(-speed);
//   BR_motor.setPWM(speed*-1);
// }
// void moveDiagFL(float speed){
//   FL_motor.setPWM(0);
//   FR_motor.setPWM(speed*-1);
//   BL_motor.setPWM(speed);
//   BR_motor.setPWM(0*-1);
// }
// void moveDiagFR(float speed){
//   FL_motor.setPWM(speed);
//   FR_motor.setPWM(0*-1);
//   BL_motor.setPWM(0);
//   BR_motor.setPWM(speed*-1);
// }
// void moveDiagBR(float speed){
//   FL_motor.setPWM(0);
//   FR_motor.setPWM(-speed*-1);
//   BL_motor.setPWM(-speed);
//   BR_motor.setPWM(0*-1);
// }
// void moveDiagBL(float speed){
//   FL_motor.setPWM(-speed);
//   FR_motor.setPWM(0*-1);
//   BL_motor.setPWM(0);
//   BR_motor.setPWM(-speed*-1);
// }
// void stop(){
//   FL_motor.setPWM(0);
//   FR_motor.setPWM(0*-1);
//   BL_motor.setPWM(0);
//   BR_motor.setPWM(0*-1);
// }
// void rotate(float speed, int direction, int point_of_rotation){
//   float dir_speed = speed;
//   Serial.println(dir_speed);
//   if (direction < 0){
//     dir_speed *= -1;
//   }
//   if (point_of_rotation == 0){
//     FL_motor.setPWM(dir_speed);
//     FR_motor.setPWM(-dir_speed*-1);
//     BL_motor.setPWM(dir_speed);
//     BR_motor.setPWM(-dir_speed*-1);
//   }
//   else if (point_of_rotation == 1){
//     FL_motor.setPWM(dir_speed);
//     FR_motor.setPWM(-dir_speed*-1);
//     BL_motor.setPWM(0);
//     BR_motor.setPWM(0*-1);
//   }
//   else if (point_of_rotation == 2){
//     FL_motor.setPWM(0);
//     FR_motor.setPWM(0*-1);
//     BL_motor.setPWM(dir_speed);
//     BR_motor.setPWM(-dir_speed*-1);
//   }
//   else if (point_of_rotation == 3){
//     FL_motor.setPWM(dir_speed);
//     FR_motor.setPWM(0*-1);
//     BL_motor.setPWM(dir_speed);
//     BR_motor.setPWM(0*-1);
//   }
//   else if (point_of_rotation == 4){
//     FL_motor.setPWM(0);
//     FR_motor.setPWM(dir_speed*-1);
//     BL_motor.setPWM(0);
//     BR_motor.setPWM(dir_speed*-1);
//   }
// }

// void demo(){
//   float speed = 0.5;
//   int stop_time = 50;
//   int move_time = 1000;

//   // FL Square Loop
//   moveForward(speed);
//   delay(move_time);
//   stop();
//   delay(stop_time);
//   moveLeft(speed);
//   delay(move_time);
//   stop();
//   delay(stop_time);
//   moveBackward(speed);
//   delay(move_time);
//   stop();
//   delay(stop_time);
//   moveRight(speed);
//   delay(move_time);
//   stop();
//   delay(stop_time);

//   // BR Square Loop
//   moveRight(speed);
//   delay(move_time);
//   stop();
//   delay(stop_time);
//   moveBackward(speed);
//   delay(move_time);
//   stop();
//   delay(stop_time);
//   moveLeft(speed);
//   delay(move_time);
//   stop();
//   delay(stop_time);
//   moveForward(speed);
//   delay(move_time);
//   stop();
//   delay(stop_time);

//   // FL Diag Loop
//   moveDiagFL(speed);
//   delay(move_time);
//   stop();
//   delay(stop_time);
//   moveDiagBL(speed);
//   delay(move_time);
//   stop();
//   delay(stop_time);
//   moveDiagBR(speed);
//   delay(move_time);
//   stop();
//   delay(stop_time);
//   moveDiagFR(speed);
//   delay(move_time);
//   stop();
//   delay(stop_time);

//   // FR Diag Loop
//   moveDiagFR(speed);
//   delay(move_time);
//   stop();
//   delay(stop_time);
//   moveDiagBR(speed);
//   delay(move_time);
//   stop();
//   delay(stop_time);
//   moveDiagBL(speed);
//   delay(move_time);
//   stop();
//   delay(stop_time);
//   moveDiagFL(speed);
//   delay(move_time);
//   stop();
//   delay(stop_time);

//   // Clockwise, center
//   rotate(speed,1,0);
//   delay(move_time);
//   stop();
//   delay(stop_time);

//   // CouterClockwise, center
//   rotate(speed,-1,0);
//   delay(move_time);
//   stop();
//   delay(stop_time);

//   // Clockwise, back
//   rotate(speed,1,1);
//   delay(move_time);
//   stop();
//   delay(stop_time);

//   // ?
//   rotate(speed,-1,4);
//   delay(move_time);
//   stop();
//   delay(stop_time);

// }