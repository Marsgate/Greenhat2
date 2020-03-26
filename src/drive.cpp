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
const int accel_step = 8; //smaller number = more slew
const int deccel_step = 200; //200 = no slew
const int arc_step = 2; // acceleration for arcs

//straight driving constants
const double driveKP = .3;
const double driveKD = .5;

//turning constants
const double turnKP = .8;
const double turnKD = 3;

//arc constants
const double arcKP = .05;


/**************************************************/
//edit below with caution!!!
static int driveMode = 0;
static int driveTarget = 0;
static int turnTarget = 0;
static int maxSpeed = 100;

//gyro
inertial iSens(gyro_port);

//motors
motor left1(left_front, ratio18_1, 0);
motor left2(left_rear, ratio18_1, 0);
motor right1(right_front, ratio18_1, 1);
motor right2(right_rear, ratio18_1, 1);

motor_group leftMotors = {left1, left2};
motor_group rightMotors = {right1, right2};

/**************************************************/
//task::sleep shorthand
void delay(int sleepTime){
  task::sleep(sleepTime);
}

/**************************************************/
//basic control
void left_drive(int vel){
  vel *= 120;
  leftMotors.spin(fwd, vel, voltageUnits::mV);
}

void right_drive(int vel){
  vel *= 120;
  rightMotors.spin(fwd, vel, voltageUnits::mV);
}

void setBrakeMode(brakeType b){
  leftMotors.setStopping(b);
  rightMotors.setStopping(b);
  leftMotors.stop();
  rightMotors.stop();
}

void reset(){
  leftMotors.resetRotation();
  rightMotors.resetRotation();
  setBrakeMode(coast);
}

int drivePos(){
  return (rightMotors.rotation(deg) + leftMotors.rotation(deg))/2;
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
//drive settling
bool isDriving(){
  static int count = 0;
  static int last = 0;
  static int lastTarget = 0;

  int curr = drivePos();

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

void waitUntilSettled(){
  while(isDriving()) delay(10);
}

/**************************************************/
//autonomous functions
void driveAsync(double sp, int max){
  sp *= distance_constant;
  reset();
  maxSpeed = max;
  driveTarget = sp;
  driveMode = 1;
}

void turnAsync(double sp, int max){
  sp *= degree_constant;
  reset();
  maxSpeed = max;
  turnTarget = sp;
  driveMode = -1;
}

void drive(double sp, int max){
  driveAsync(sp, max);
  delay(450);
  while(isDriving()) delay(20);
}

void turn(double sp, int max){
  turnAsync(sp, max);
  delay(450);
  while(isDriving()) delay(20);
}

void fastDrive(double sp, int max){
  if(sp < 0) max = -max;
  reset();
  lastSpeed = max;
  driveMode = 0;
  left_drive(max);
  right_drive(max);

  if(sp > 0)
    while(drivePos() < sp * distance_constant) delay(20);
  else
    while(drivePos() > sp * distance_constant) delay(20);
}

void timeDrive(int t, int left, int right){
  left_drive(left);
  right_drive(right == 0 ? left : right);
  delay(t);
}

void velocityDrive(int t, int max){
  leftMotors.spin(fwd, max, pct);
  rightMotors.spin(fwd, max, pct);
  delay(t);
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
  if(type != 2){
    leftMotors.stop();
    rightMotors.stop();
    delay(10);
  }

  while(time_step < arc_length){

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
    if(speed < 0)
      speed = 0;

    speed = slew(speed); //slew

    if(reversed)
      speed = -speed;

    double scaled_speed = speed*rad;

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

  if(type != 1){
    leftMotors.stop();
    rightMotors.stop();
  }
}

void arcLeft(int arc_length, double rad, int max, int type){
  arc(false, arc_length, rad, max, type);
}

void arcRight(int arc_length, double rad, int max, int type){
  arc(true, arc_length, rad, max, type);
}

void scurve(bool mirror, int arc1, int mid, int arc2, int max){

  //first arc
  arc(mirror, arc1, 0, max, 1);
 
  //middle movement
  velocityDrive(mid, max);

  //final arc
  arc(!mirror, arc2, 0, max, 2);

}

void sLeft(int arc1, int mid, int arc2, int max){
  scurve(false, arc1, mid, arc2, max);
}

void sRight(int arc1, int mid, int arc2, int max){
  scurve(true, arc1, mid, arc2, max);
}

void _sLeft(int arc1, int mid, int arc2, int max){
  scurve(true, -arc1, mid, -arc2, -max);
}

void _sRight(int arc1, int mid, int arc2, int max){
  scurve(false, -arc1, -mid, -arc2, max);
}

/**************************************************/
//task control
int driveTask(){
  int prevError = 0;
  double kp;
  double kd;
  int sp;

  while(1){
    delay(20);

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
    int sv = (right1.rotation(deg) + left1.rotation(deg)*driveMode)/2;
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

/**************************************************/
//operator control
void tank(int left, int right){
  driveMode = 0; //turns off autonomous tasks
  left_drive(left);
  right_drive(right);
}

void arcade(int vertical, int horizontal){
  driveMode = 0; //turns off autonomous task

  left_drive(vertical + horizontal);
  right_drive(vertical - horizontal);
}