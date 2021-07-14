#include "main.h"

//Motor and controller declarations
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor driveFL(14);
pros::Motor driveBL(11);
pros::Motor driveFR(16, true);
pros::Motor driveBR(13, true);
pros::Motor lift(2);
