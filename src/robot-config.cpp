#include "vex.h"
using namespace vex;

brain  Brain;
controller Controller1 = controller(primary);

//insert custom subsystems below

//example intake subsystem
motor intake_motor1 = motor(PORT5, ratio18_1, false);
motor intake_motor2 = motor(PORT6, ratio18_1, true);
motor_group intake(intake_motor1, intake_motor2);
Pcontroller intakeController(intake, 0.1);

void vexcodeInit( void ) {
  initDrive();
  intake.setStopping(hold);
}