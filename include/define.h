#include "main.h"

#ifndef DEFINE_H
#define DEFINE_H

//Allows all parts of the program to access the motors and controller as global variables
extern pros::Controller master;
extern pros::Motor driveFL;
extern pros::Motor driveML;
extern pros::Motor driveBL;
extern pros::Motor driveFR;
extern pros::Motor driveMR;
extern pros::Motor driveBR;
extern pros::Motor smallLift;
extern pros::Motor bigLift1;
extern pros::Motor bigLift2;
extern pros::Motor intake;

extern pros::ADIDigitalOut frontClip;

extern pros::Rotation leftEncoder;
extern pros::Rotation rightEncoder;
extern pros::ADIEncoder backEncoder;

extern pros::Imu imu;

extern pros::task_t smallLiftTask;
extern pros::task_t bigLiftTask;
extern pros::task_t driveTrainTask;
extern pros::task_t purePursuitTask;
extern pros::task_t odometryTask;

extern float smallLiftSetpoint;
extern float smallLiftKP;
extern float bigLiftSetpoint;

extern pros::Motor driveTrain[6];
extern std::array<pros::Motor, 3> driveTrainL;
extern std::array<pros::Motor, 3> driveTrainR;

extern pros::Rotation encoders[2];


typedef void(*FnPtr) ();
extern void (*grabL) (), (*grabR) (), (*winPointL) (), (*winPointR) (), (*fullL) (), (*none) ();
extern FnPtr autonPointers[];
extern int autonSelect;

extern double wheelConversionFactor;

extern FILE* targetVelocityL;
extern FILE* targetVelocityR;
extern FILE* measuredVelocityL;
extern FILE* measuredVelocityR;

extern double DPStoRPM;

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
