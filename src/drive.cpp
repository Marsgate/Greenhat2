#include "vex.h"
using namespace vex;

/**************************************************/
//user editable constants

//motor ports
const int left_front = PORT1;
const int left_rear = PORT2;
const int right_front = PORT3;
const int right_rear = PORT4;

//gyro port (set to 0 if not using)
const int gyro_port = 0;

//distance constants
const int distance_constant = 545; //ticks per tile
const double degree_constant = 2.3; //ticks per degree

/**************************************************/
//advanced tuning (PID and slew)

//slew control (autonomous only)
const int arc_step = 3;
const int accel_step = 8; //smaller number = more slew
const int deccel_step = 200; //200 = no slew

//straight driving constants
const double driveKP = .3;
const double driveKD = .5;

//turning constants
const double turnKP = .8;
const double turnKD = 3;

//arc constants
const double arcKP = .15;


/**************************************************/
//edit below with caution!!!
static int driveMode = 0;
static int driveTarget = 0;
static int turnTarget = 0;
static int maxSpeed = 100;

//gyro
inertial iSens(gyro_port);

//motors
motor left1(left_front, gearSetting::ratio18_1, 0);
motor left2(left_rear, gearSetting::ratio18_1, 0);
motor right1(right_front, gearSetting::ratio18_1, 1);
motor right2(right_rear, gearSetting::ratio18_1, 1);

motor_group leftMotors = {left1, left2}, rightMotors = {right1, right2};

/**************************************************/
//basic control
void left_drive(int vel){
  vel *= 120;
  leftMotors.spin(directionType::fwd, vel, voltageUnits::mV);
}

void right_drive(int vel){
  vel *= 120;
  rightMotors.spin(directionType::fwd, vel, voltageUnits::mV);
}

void timeDrive(int t, int speed){
  left_drive(speed);
  right_drive(speed);
  delay(t);
}

void reset(){
  left1.resetRotation();
  left2.resetRotation();
  right1.resetRotation();
  right2.resetRotation();
}

int drivePos(){
  return (left1.rotation(rotationUnits::deg) + left2.rotation(rotationUnits::deg))/2;
}

/**************************************************/
//slew control
static int lastSpeed = 0;
int slew(int speed){
  int step;

  if(abs(lastSpeed) < abs(speed))
    if(driveMode == 0)
      step = arc_step;
    else
      step = accel_step;
  else
    step = deccel_step;

  if(speed > lastSpeed + step)
    lastSpeed += step;
  else if(speed < lastSpeed - step)
    lastSpeed -= step;
  else{
    lastSpeed = speed;
  }

  return lastSpeed;
}

/**************************************************/
//feedback
bool isDriving(){
  static int count = 0;
  static int last = 0;
  static int lastTarget = 0;

  int curr = left2.rotation(rotationUnits::deg);

  int target = turnTarget;
  if(driveMode == 1)
    target = driveTarget;

  if(abs(last-curr) < 3)
    count++;
  else
    count = 0;

  if(target != lastTarget)
    count = 0;

  lastTarget = target;
  last = curr;

  //not driving if we haven't moved
  if(count > 4)
    return false;
  else
    return true;
}

/**************************************************/
//drive modifiers
void setSpeed(int speed){
  maxSpeed = speed;
}

void setBrakeMode(brakeType b){
  left1.setStopping(b);
  left2.setStopping(b);
  right1.setStopping(b);
  right2.setStopping(b);
  left1.stop();
  left2.stop();
  right1.stop();
  right2.stop();
}

/**************************************************/
//autonomous functions
void driveAsync(double sp){
  sp *= distance_constant;
  reset();
  driveTarget = sp;
  driveMode = 1;
}

void turnAsync(double sp){
  sp *= degree_constant;
  reset();
  turnTarget = sp;
  driveMode = -1;
}

void drive(double sp, int speed){
  driveAsync(sp);
  setSpeed(speed);
  task::sleep(450);
  while(isDriving()) task::sleep(20);
}

void turn(double sp, int speed){
  turnAsync(sp);
  setSpeed(speed);
  task::sleep(450);
  while(isDriving()) task::sleep(20);
}

void fastDrive(double sp, int speed){
  if(sp < 0) speed = -speed;
  reset();
  lastSpeed = speed;
  driveMode = 0;
  left_drive(speed);
  right_drive(speed);

  if(sp > 0)
    while(drivePos() < sp * distance_constant) task::sleep(20);
  else
    while(drivePos() > sp * distance_constant) task::sleep(20);
}

void arc(bool mirror, int arc_length, double rad, int max, int type){
  reset();
  int time_step = 0;
  driveMode = 0;
  bool reversed = false;

  //reverse the movement if the length is negative
  if(arc_length < 0){
    reversed = true;
    arc_length = -arc_length;
  }

  //fix jerk bug between velocity movements
  leftMotors.stop();
  rightMotors.stop();
  delay(10);

  while(isDriving() || time_step < 25){

    //speed
    int error = arc_length-time_step;
    int speed = error*arcKP;

    if(type == 1)
      speed = max;

    //speed limiting
    if(speed > max)
      speed = max;
    if(speed < -max)
      speed = -max;

    //prevent backtracking
    if(arc_length > 0){
      if(speed < 0)
        speed = 0;
    }else{
      if(speed > 0)
        speed = 0;
    }

    speed = slew(speed); //slew

    if(reversed)
      speed = -speed;

    double scaled_speed = rad;

    if(type == 1)
      scaled_speed = speed * (double)time_step/arc_length;
    else if(type == 2)
      scaled_speed = speed * (1-(double)time_step/arc_length);

    //assign drive motor speeds
    leftMotors.spin(fwd, mirror ? speed : scaled_speed, pct);
    rightMotors.spin(fwd, mirror ? scaled_speed : speed, pct);

    //increment time step
    time_step += 10;
    delay(10);
  }
}

void arcLeft(int arc_length, double rad, int max, int type){
  arc(false, arc_length, rad, max, type);
}

void arcRight(int arc_length, double rad, int max, int type){
  arc(true, arc_length, rad, max, type);
}

void scurve(bool mirror, int arc1, int mid, int arc2, int max){

  //scaling based on max speed;
  arc1 *= (float)40/max;
  mid *= (float)40/max;
  arc2 *= (float)40/max;

  arc(mirror, arc1, 0, max, 1);
 
  //middle movement
  leftMotors.spin(fwd, max, pct);
  rightMotors.spin(fwd, max, pct);
  delay(mid);

  //final arc
  arc(!mirror, arc2, 0, max, 2);

}

void sLeft(int arc1, int mid, int arc2, int max){
  scurve(false, arc1, mid, arc2, max);
}

void sRight(int arc1, int mid, int arc2, int max){
  scurve(true, arc1, mid, arc2, max);
}

/**************************************************/
//task control
int driveTask(){
  int prevError = 0;
  double kp;
  double kd;
  int sp;

  while(1){
    task::sleep (20);

    if(driveMode == 1){
      sp = driveTarget;
      kp = driveKP;
      kd = driveKD;
    }else if(driveMode == -1){
      sp = turnTarget;
      kp = turnKP;
      kd = turnKD;
    }else{
      continue;
    }

    //read sensors
    int sv = (right1.rotation(rotationUnits::deg) + left1.rotation(rotationUnits::deg)*driveMode)/2;
    if(gyro_port != 0 && driveMode == -1){
      sv = -iSens.rotation(deg);
    }

    //speed
    int error = sp-sv;
    int derivative = error - prevError;
    prevError = error;
    int speed = error*kp + derivative*kd;

    //speed limiting
    if(speed > maxSpeed)
      speed = maxSpeed;
    if(speed < -maxSpeed)
      speed = -maxSpeed;

    speed = slew(speed); //slew

    //set motors
    left_drive(speed*driveMode);
    right_drive(speed);
  }
}

void initDrive(){
  task drive_task(driveTask);
  if(gyro_port != 0){
    iSens.calibrate();
    while(iSens.isCalibrating()) delay(20);
  }
}

void delay(int sleepTime){
  task::sleep(sleepTime);
}

/**************************************************/
//operator control
void tankOp(){
  driveMode = 0; //turns off autonomous tasks
  int lJoy = Controller1.Axis3.position();
  int rJoy = Controller1.Axis2.position();

  left_drive(lJoy);
  right_drive(rJoy);
}

void arcadeOp(){
  driveMode = 0; //turns off autonomous tasks
  int vJoy = Controller1.Axis3.position();
  int hJoy = Controller1.Axis4.position();

  left_drive(vJoy + hJoy);
  right_drive(vJoy - hJoy);
}