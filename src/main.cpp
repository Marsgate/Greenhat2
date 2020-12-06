#include "vex.h"
using namespace vex;

competition Competition;

void pre_auton(void) {
  selector::init();
  vexcodeInit();
}


int autonTimer(){
  wait(60, sec);
  Competition.test_disable();
  return 0;
}

void usercontrol(void) {
  while (1) {

    tank(Controller1.Axis3.position(),
      Controller1.Axis2.position());

    if(Controller1.ButtonR1.pressing()){
      intake.spin(fwd, 100, pct);
    }else if(Controller1.ButtonR2.pressing()){
      intake.spin(rev, 50, pct);
    }else{
      intake.stop();
    }

    //autonomous button (starts auton from driver control for testing purposes)
    if(Controller1.ButtonLeft.pressing() && !Competition.isFieldControl() && !Competition.isCompetitionSwitch()){
      //task auton_timer(autonTimer);
      autonomous();
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
