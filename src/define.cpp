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

//Pneumatics delcarations
pros::ADIDigitalOut frontClip(2);
pros::ADIDigitalOut backClip(1);

//Sensor declarations
pros::Rotation leftEncoder(2);
pros::Rotation rightEncoder(18);
pros::ADIEncoder backEncoder(3, 4, true);

pros::Imu imu(15);

/*
//Multithreading task declaration
pros::task_t smallLiftTask;
pros::task_t bigLiftTask;
pros::task_t driveTrainTask;
pros::task_t purePursuitTask;
pros::task_t odometryTask;
*/


//Declaration of drive train/sensor arrays for easier access
pros::Motor driveTrain[6] {driveFL, driveBR, driveBL, driveFR, driveML, driveMR};
std::array<pros::Motor, 3> driveTrainL = {driveFL, driveML, driveBL};
std::array<pros::Motor, 3> driveTrainR = {driveFR, driveMR, driveBR};


//Auton selector
typedef void(*FnPtr) ();
void (*grabL) (){&LeftGrab}, (*grabR) (){&RightGrab}, (*winPointL) (){&LeftWinPoint}, (*winPointR) (){&RightWinPoint}, (*fullL) (){&LeftFull}, (*fullR) (){&RightFull}, (*dGrab) (){&DoubleGrab}, (*wP) (){&FullWinPoint}, (*none) (){None};
FnPtr autonPointers[] {none, none, none, none, fullL, wP, fullR, grabL, grabR, winPointL, dGrab, winPointR, dGrab};
int autonSelect = 0;

//File I/O for data output
FILE* targetVelocityL = fopen("/usd/telem/targetVelocityTelemL.txt", "w");
FILE* targetVelocityR = fopen("/usd/telem/targetVelocityTelemR.txt", "w");
FILE* measuredVelocityL = fopen("/usd/telem/measuredVelocityTelemL.txt", "w");
FILE* measuredVelocityR = fopen("/usd/telem/measuredVelocityTelemR.txt", "w");

//Convert between degrees per second and rotations per minute
double DPStoRPM = 0.166666667;

//Definition of struct "position" for use in odometry code
sPos position;

//Conversion factor from degrees of wheel rotation to inches
double wheelConversionFactor = 28.648;
