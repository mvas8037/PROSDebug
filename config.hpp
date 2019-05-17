#include "main.h"
#ifndef CONFIG_H

#define CONFIG_H

extern pros::Motor lfront ;
extern pros::Motor lback ;
extern pros::Motor rfront ;
extern pros::Motor rback ;

extern pros::Controller Controller1; // master control
//extern pros::Controller Controller2; //partner control
extern pros::ADIEncoder encoder;
extern pros::ADIGyro gyro;

//method headers
void pLoop (int direction, double target);
void drive (double unit);
void robotMove (int direction, double target);
void baseHold (void);
void driveTrain (float unit);

#endif
