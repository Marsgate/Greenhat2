#include "vex.h"
using namespace vex;

void redAuton(){

}

void blueAuton(){
    
}

void autonSelector(){
  if(Brain.ThreeWirePort.A.value()){
    redAuton();
  }else if (Brain.ThreeWirePort.B.value()){
    blueAuton();
  }
}