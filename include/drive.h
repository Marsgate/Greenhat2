using namespace vex;

void reset();
int drivePos();
bool isDriving();

int driveTask();
int turnTask();

void driveAsync(double sp);
void turnAsync(double sp);
void drive(double sp, int speed = 100);
void turn(double sp, int speed = 100);
void fastDrive(double sp, int speed = 100);

void setSpeed(int speed);
void setBrakeMode(vex::brakeType b);

void tankOp();
void arcadeOp();

void initDrive();

void delay(int sleepTime);