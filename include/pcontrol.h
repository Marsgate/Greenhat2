#include <cmath>
using namespace vex;

class Pcontroller{
  double _target;
  double _lastTarget = 0;
  double _count = 0;
  double _last = 0;
  double _kp;
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
  bool isMoving(){
    int curr = _mg.rotation(rotationUnits::deg);

    if(std::abs(_last-curr) < 3)
      _count++;
    else
      _count = 0;

    if(_target != _lastTarget)
      _count = 0;

    _lastTarget = _target;
    _last = curr;

    if(_count > 4)
      return false;
    else
      return true;
  }
  void spin(double target){
    _target = target;
    _mg.spin(fwd, getOutput(), voltageUnits::mV);
  }
  void spinTo(double target){
    wait(250, msec);
    while(isMoving()){
      spin(target);
      wait(20, msec);
    }
    _mg.stop();
  }
  double getOutput(){
    double sv = _mg.position(deg);
    double speed = (_target-sv)*_kp*120;
    if(speed > 0 && speed < _kf){
      speed = _kf;
    }else if(speed < 0 && speed > _kf){
      speed = -_kf;
    }
    return speed;
  }
};