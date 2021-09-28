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
pros::Motor intake(12, pros::E_MOTOR_GEARSET_06);

pros::ADIDigitalOut clip(3);

pros::task_t smallLiftTask;
pros::task_t bigLiftTask;

float smallLiftSetpoint = 0;
float bigLiftSetpoint = 0;

sPos position;
