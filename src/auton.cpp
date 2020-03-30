#include "vex.h"
using namespace vex;

void redAuton(){

}

void blueAuton(){
    
}

void autonomous(){
  if(selector::auton == 1){
    redAuton();
  }else if (selector::auton == 2){
    blueAuton();
  }
}