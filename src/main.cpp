#include "vex.h"
using namespace vex;

competition Competition;

void pre_auton(void) {
  vexcodeInit();
}

void autonomous(void) {
  autonSelector();
}

void usercontrol(void) {
  while (1) {
    if(Controller1.ButtonR1.pressing()){
      intake.spin(fwd); //default is 100%
    }else if(Controller1.ButtonR2.pressing()){
      intake.spin(rev, 50, pct);
    }else{
      intake.stop();
    }

    delay(20);
  }
}

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
