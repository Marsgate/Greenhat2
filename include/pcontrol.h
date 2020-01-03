#include <cmath>
using namespace vex;

class Pcontroller{
  double _target;
  double _kp;
  double _speed;
  double _margin;
  double _kf;
  motor_group _mg;

  public:
  Pcontroller(motor_group mg, double kp, double kf = 15, double margin = 10){
    _kp = kp;
    _mg = mg;
    _margin = margin;
    _kf = kf;
  }
  void spin(double target){
    _target = target;
    _mg.spin(fwd, getOutput(), voltageUnits::mV);
  }
  void spinTo(double target){
    while(std::abs(_mg.position(deg) - target) > _margin){
      spin(target);
      wait(20, msec);
    }
    _mg.stop();
  }
  double getOutput(){
    double sv = _mg.position(deg);
    double speed = (_target-sv)*_kp*120;
    if(speed > 0 ){
      speed += _kf;
    }else{
      speed -= _kf;
    }
    return speed;
  }
  
};