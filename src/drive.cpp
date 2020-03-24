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


#define MAX 100;

static int driveMode = 1;
static int driveTarget = 0;
static int turnTarget = 0;
static int maxSpeed = MAX;

//gyro
inertial iSens(gyro_port);

//motors
motor left1(left_front, gearSetting::ratio6_1, 0);
motor left2(left_rear, gearSetting::ratio6_1, 0);
motor right1(right_front, gearSetting::ratio6_1, 1);
motor right2(right_rear, gearSetting::ratio6_1, 1);

/**************************************************/
//basic control
void left_drive(int vel){
  vel *= 120;
  left1.spin(directionType::fwd, vel, voltageUnits::mV);
  left2.spin(directionType::fwd, vel, voltageUnits::mV);
}

void right_drive(int vel){
  vel *= 120;
  right1.spin(directionType::fwd, vel, voltageUnits::mV);
  right2.spin(directionType::fwd, vel, voltageUnits::mV);
}

void timeDrive(int t, int speed){
  left_drive(speed);
  right_drive(speed);
  delay(t);
}

void reset(){
  driveMode = 0;
  left1.resetRotation();
  left2.resetRotation();
  right1.resetRotation();
  right2.resetRotation();
}

int drivePos(){
  return (left1.rotation(rotationUnits::deg) + left2.rotation(rotationUnits::deg))/2;
}

/**************************************************/
//Velocity control
void left_drive_vel(int vel){
  left1.spin(fwd, vel, pct);
  left2.spin(fwd, vel, pct);
}

void right_drive_vel(int vel){
  right1.spin(fwd, vel, pct);
  right2.spin(fwd, vel, pct);
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

void arc(bool mirror, int arc_length, double rad, int max){
  reset();
  int time_step = 0;
  driveMode = 0;

  //fix jerk bug between velocity movements
  left_drive_vel(0);
  right_drive_vel(0);
  delay(10);

  while(isDriving() || time_step < 250){

    //speed
    int error = arc_length-time_step;
    int speed = error*arcKP;

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

    int left_speed = speed;
    int right_speed = speed;
    
    if(!mirror)
      left_speed *= rad;
    else
      right_speed *= rad;

    //assign drive motor speeds
    left_drive_vel(left_speed);
    right_drive_vel(right_speed);

    //increment time step
    time_step += 10;
    delay(10);
  }
}

void arcLeft(int arc_length, double rad, int max){
  arc(false, arc_length, rad, max);
}

void arcRight(int arc_length, double rad, int max){
  arc(true, arc_length, rad, max);
}

void scurve(bool mirror, int arc1, int mid, int arc2, int max){
  reset();
  int time_step = 0;
  driveMode = 0;

  //fix jerk bug between velocity movements
  left_drive_vel(0);
  right_drive_vel(0);
  delay(10);

  if(arc2 == 0)
    arc2 = arc1;

  //scaling based on max speed;
  arc1 *= (float)40/max;
  mid *= (float)40/max;
  arc2 *= (float)40/max;

  arc2 += 150;

  //first arc
  while (time_step < arc1){

    int speed = slew(max); //slew

    int left_speed = speed;
    int right_speed = speed;

    double pctComplete = (double)time_step/arc1;
    double scaled_rad = pctComplete;

    if(!mirror)
      left_speed *= scaled_rad;
    else
      right_speed *= scaled_rad;

    //assign drive motor speeds
    left_drive_vel(left_speed);
    right_drive_vel(right_speed);

    time_step += 10;
    delay(10);
  }
 
  //middle movement
  time_step = 0;
  left_drive_vel(max);
  right_drive_vel(max);
  while(time_step < mid){
    time_step += 10;
    delay(10);
  }

  //final arc
  time_step = 0;
  mirror = !mirror;
  while(isDriving() || time_step < 250){

    //speed
    int error = arc2-time_step;
    int speed = error*arcKP;

    //speed limiting
    if(speed > max)
      speed = max;
    if(speed < -max)
      speed = -max;

    //prevent backtracking
    if(arc2 > 0){
      if(speed < 0)
        speed = 0;
    }else{
      if(speed > 0)
        speed = 0;
    }

    speed = slew(speed); //slew

    int left_speed = speed;
    int right_speed = speed;
    
    double pctComplete = (double)time_step/arc2;
    double scaled_rad = (1-pctComplete);

    if(!mirror)
      left_speed *= scaled_rad;
    else
      right_speed *= scaled_rad;

    //assign drive motor speeds
    left_drive_vel(left_speed);
    right_drive_vel(right_speed);

    //increment time step
    time_step += 10;
    delay(10);
  }

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