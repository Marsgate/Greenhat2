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
void timeDrive(int t, int speed = 100);

void arcLeft(int length, double rad, int max = 100, int type = 0);
void arcRight(int length, double rad, int max = 100, int type = 0);
void sLeft(int arc1, int mid, int arc2, int max = 100);
void sRight(int arc1, int mid, int arc2, int max = 100);
void _sLeft(int arc1, int mid, int arc2, int max = 100);
void _sRight(int arc1, int mid, int arc2, int max = 100);

void setSpeed(int speed);
void setBrakeMode(brakeType b);

void tankOp();
void arcadeOp();

void initDrive();

void delay(int sleepTime);