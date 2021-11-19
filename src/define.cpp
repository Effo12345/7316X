#include "main.h"
#include "define.h"
#include "autonomous.h"
//Motor and controller declarations
//Port 5 is broken
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor driveFL(20, true);
pros::Motor driveML(19);
pros::Motor driveBL(16, true);
pros::Motor driveFR(1);
pros::Motor driveMR(8, true);
pros::Motor driveBR(4);
pros::Motor lift(6);
pros::Motor intake(3, pros::E_MOTOR_GEARSET_06, true);

pros::ADIDigitalOut frontClip(2);
pros::ADIDigitalOut backClip(1);

pros::Rotation leftEncoder(2);
pros::Rotation rightEncoder(18);
pros::ADIEncoder backEncoder(3, 4, true);

pros::Imu imu(15);

/*
pros::task_t smallLiftTask;
pros::task_t bigLiftTask;
pros::task_t driveTrainTask;
pros::task_t purePursuitTask;
pros::task_t odometryTask;
*/

float bigLiftSetpoint = 0;

pros::Motor driveTrain[6] {driveFL, driveBR, driveBL, driveFR, driveML, driveMR};
std::array<pros::Motor, 3> driveTrainL = {driveFL, driveML, driveBL};
std::array<pros::Motor, 3> driveTrainR = {driveFR, driveMR, driveBR};

pros::Rotation encoders[2] {leftEncoder, rightEncoder};

//Auton selector
typedef void(*FnPtr) ();
void (*grabL) (){&LeftGrab}, (*grabR) (){&RightGrab}, (*winPointL) (){&LeftWinPoint}, (*winPointR) (){&RightWinPoint}, (*fullL) (){&LeftFull}, (*fullR) (){&LeftFull}, (*dGrab) (){&DoubleGrab}, (*fGrabL) (){&FullGrab}, (*none) (){None};
FnPtr autonPointers[] {none, none, none, none, fullL, fGrabL, fullR, grabL, grabR, winPointL, dGrab, winPointR, dGrab};
int autonSelect = 0;

FILE* targetVelocityL = fopen("/usd/telem/targetVelocityTelemL.txt", "w");
FILE* targetVelocityR = fopen("/usd/telem/targetVelocityTelemR.txt", "w");
FILE* measuredVelocityL = fopen("/usd/telem/measuredVelocityTelemL.txt", "w");
FILE* measuredVelocityR = fopen("/usd/telem/measuredVelocityTelemR.txt", "w");

double DPStoRPM = 0.166666667;

sPos position;

double wheelConversionFactor = 28.648;
