#include "main.h"
#include "define.h"
//Motor and controller declarations
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor driveFL(15);
pros::Motor driveBL(17);
pros::Motor driveFR(18, true);
pros::Motor driveBR(16, true);
pros::Motor smallLift(2);
pros::Motor bigLift1(10);
pros::Motor bigLift2(19, true);
pros::Motor intake(12);

pros::ADIDigitalOut clip(3);

pros::Rotation leftEncoder(11);
pros::Rotation rightEncoder(13);
pros::ADIEncoder backEncoder(1, 2, true);

pros::Imu imu(14);

pros::task_t smallLiftTask;
pros::task_t bigLiftTask;
pros::task_t driveTrainTask;
pros::task_t purePursuitTask;
pros::task_t odometryTask;

float smallLiftSetpoint = 0;
float smallLiftKP;
float bigLiftSetpoint = 0;
float driveTrainSetpoint = 0;
float driveError = 6;
float driveTrainKP;
float driveLeftError;
float driveRightError;
float driveLeftSetpoint;
float driveRightSetpoint;

int autonSelect = 0;

FILE* targetVelocityL = fopen("/usd/telem/targetVelocityTelemL.txt", "w");
FILE* targetVelocityR = fopen("/usd/telem/targetVelocityTelemR.txt", "w");
FILE* measuredVelocityL = fopen("/usd/telem/measuredVelocityTelemL.txt", "w");
FILE* measuredVelocityR = fopen("/usd/telem/measuredVelocityTelemR.txt", "w");

double DPStoRPM = 0.166666667;

sPos position;

double wheelConversionFactor = 28.648;
