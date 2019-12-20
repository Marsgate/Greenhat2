#include <cmath>
using namespace vex;

class Pcontroller{
  double _target;
  double _kp;
  double _speed;
  double _margin;
  motor_group _mg;

  public:
  Pcontroller(motor_group mg, double kp, double margin = 3){
    _kp = kp;
    _mg = mg;
    _margin = margin;
  }
  void spin(double target){
    _target = target;
    _mg.spin(fwd, getOutput(), voltageUnits::mV);
  }
  void spinTo(double target){
    while(std::abs(_mg.position(deg)) - std::abs(_target) > _margin){
      _target = target;
      _mg.spin(fwd, getOutput(), voltageUnits::mV);
    }
    _mg.stop();
  }
  double getOutput(){
    double sv = _mg.position(deg);
    double speed = (_target-sv)*_kp*120;
    return speed;
  }
  
};