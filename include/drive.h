using namespace vex;

void delay(int sleepTime);

void setBrakeMode(brakeType b);
void reset();
int drivePos();

bool isDriving();
void waitUntilSettled();

void driveAsync(double sp, int max = 100);
void turnAsync(double sp, int max = 100);
void drive(double sp, int max = 100);
void turn(double sp, int max = 100);
void fastDrive(double sp, int max = 100);
void timeDrive(int t, int left = 100, int right = 0);
void velocityDrive(int t, int max = 100);

void arcLeft(int length, double rad, int max = 100, int type = 0);
void arcRight(int length, double rad, int max = 100, int type = 0);
void sLeft(int arc1, int mid, int arc2, int max = 100);
void sRight(int arc1, int mid, int arc2, int max = 100);
void _sLeft(int arc1, int mid, int arc2, int max = 100);
void _sRight(int arc1, int mid, int arc2, int max = 100);

int driveTask();
int turnTask();

void tank(int left, int right);
void arcade(int vertical, int horizontal);

void initDrive();