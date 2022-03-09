// #include <Arduino.h>
// #include <ros.h>
// #include <base_control/Rufus_base_msgs.h>

// ros::NodeHandle nh;

// void commandCB(const base_control::Rufus_base_msgs& motor_cmd)
// {
//   float cmd_FL = motor_cmd.motor_FL;
//   float cmd_FR = motor_cmd.motor_FR;
//   float cmd_BL = motor_cmd.motor_BL;
//   float cmd_BR = motor_cmd.motor_BR;
  
//   //Ajouter les fonctions pour faire tourner les moteurs en fonction des cmd_XX

// }

// ros::Subscriber<base_control::Rufus_base_msgs> motor_sub("/rufus/base_arduino", commandCB);

// void setup() {
//   // put your setup code here, to run once:
  
//   nh.initNode();
//   nh.subscribe(motor_sub);
// }

// void loop() {
//   nh.spinOnce();
//   delay(10);
// }
