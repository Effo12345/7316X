#include "main.h"

#ifndef DEFINE_H
#define DEFINE_H

//Allows all parts of the program to access the motors and controller as global variables
extern pros::Controller master;
extern pros::Motor driveFL;
extern pros::Motor driveBL;
extern pros::Motor driveFR;
extern pros::Motor driveBR;
extern pros::Motor smallLift;
extern pros::Motor bigLift1;
extern pros::Motor bigLift2;
extern pros::Motor intake;

extern pros::ADIDigitalOut clip;

extern pros::task_t smallLiftTask;
extern pros::task_t bigLiftTask;

extern float smallLiftSetpoint;
extern float bigLiftSetpoint;


struct sPos {
  float leftLst;
  float rightLst;
  float backLst;

  float a;
  float x;
  float y;
} ;
extern sPos position;

struct vector
{
    float x;
    float y;
    float magnitude;
};

template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

#endif
