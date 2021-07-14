#include "main.h"

#ifndef DEFINE_H
#define DEFINE_H

//Allows all parts of the program to access the motors and controller as global variables
extern pros::Controller master;
extern pros::Motor driveFL;
extern pros::Motor driveBL;
extern pros::Motor driveFR;
extern pros::Motor driveBR;
extern pros::Motor lift;

#endif
