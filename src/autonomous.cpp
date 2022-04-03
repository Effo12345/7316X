#include "main.h"
#include "functions.hpp"
#include "statemachine.hpp"
#include "units.hpp"
#include "odometry.hpp"
//Holds autonomous procedures

//Single neutral grabs
void LeftGrab() {
  	stateMachine.setHoldGoal();
  	stateMachine.setGuard();

  	clipGuard.set_value(true);
  	frontClip.set_value(false);
	hyperGrab(43);
	frontClip.set_value(true);
	while(Units.DegToIn(tracking.get_value()) > 10)
		drive_voltage(-12000, -12000);

	drive_voltage(0, 0);
}

void RightGrab() {
  	stateMachine.setHoldGoal();
  	stateMachine.setGuard();

  	clipGuard.set_value(true);
	hyperGrab(40);
	frontClip.set_value(true);
	translatePID(10);
}

//Primary autons
void LeftFull() {

}

void RightFull() {
  	stateMachine.setHoldGoal();
  	stateMachine.setGuard();

  	//pros::Task odom_task(TrackPosition, "Odometry");

	clipGuard.set_value(true);
	frontClip.set_value(false);
	translatePID(43);
	frontClip.set_value(true);
	lift.move_velocity(30);
	pros::delay(100);

	translatePID(22);
	lift.move_velocity(0);
	pros::delay(100);

	turnPID(-90, 0.25, 0.001, 1000);
	pros::delay(200);

	//turnTo(23, 13, true);

	goalGrab(450);
	backClip.set_value(true);
	pros::delay(100);

	intake_rot(10);
	translatePID(7);
	backClip.set_value(false);
}


//Win points
void LeftWinPoint() {
  	lift.move_velocity(10);
	backClip.set_value(true);
	pros::delay(300);
	lift.move_velocity(0);


	ringIntake(on);
	ringMove(500);
	pros::delay(2000);

	ringIntake(off);
	backClip.set_value(false);
	ringMove(500);
}

void RightWinPoint() {
  	lift.move_velocity(10);
	translatePID(11);
	lift.move_velocity(0);
	pros::delay(100);
	turnPID(-90, 160, 0.00, 1000);
	pros::delay(100);
	translatePID(-9.5);
	backClip.set_value(true);
	pros::delay(100);
	translatePID(10);
	intake_rot(10);
}

void FullWinPoint() {
	pros::Task odom_task(TrackPosition, "Odometry");

	lift.move_velocity(100);
	pros::delay(200);
	lift.move_velocity(0);
	pros::delay(500);

	//frontClip.set_value(true);
	//pros::delay(100);

	//translatePID(-10);

	//PIDTurn(90);	

	pather path1({
		{0, 0},
		{-20, -5}
	}, 5, true);

	PIDTurn(180);

	pather path2({
		{-20, -5},
		{-20, 73.5}
	}, 15, true);

	pros::delay(250);
	drive_voltage(-6000, -6000);
	pros::delay(500);

	backClip.set_value(true);
	drive_voltage(0, 0);
	pros::delay(500);

	ResetSensors(false);
	translatePID(15);

	intake_rot(10);
	pros::delay(100);

	backClip.set_value(false);
}


//Double neutral grab
void DoubleGrab() {
  	stateMachine.setHoldGoal();
  	stateMachine.setGuard();
  
  	clipGuard.set_value(true);
  	hyperGrab(50);
	frontClip.set_value(true);
	translatePID(20);
}


//Skills
void Skills() {
	pros::Task odom_task(TrackPosition, "Odometry");
	
	backClip.set_value(true);
	pros::delay(500);

	
	pather path1({
		{0.0, 0.0},
		{0.0, 10.0},
		{56.0, -2.0}
	}, 8, false);
	
	pros::delay(500);
	frontClip.set_value(true);
	pros::delay(100);

	lift.move_velocity(100);
	pros::delay(2000);
	lift.move_velocity(0);



	pather path2({
		{56.0, -2.0},
		{99.0, -30.0}
	}, 20, 150, false, 3000);

	lift.move_velocity(-100);
	pros::delay(850);
	lift.move_velocity(0);

	frontClip.set_value(false);
	lift.move_velocity(100);
	pros::delay(300);
	lift.move_velocity(0);

	drive_velocity(-100, -100);
	pros::delay(500);
	drive_velocity(0, 0);
	backClip.set_value(false);
	pros::delay(500);

	lift.move_velocity(-100);
	drive_velocity(100, 100);
	pros::delay(200);
	drive_velocity(0, 0);
	pros::delay(100);

	PIDTurn(gyro.get_rotation() + 170);
	pros::delay(500);

	drive_velocity(100, 100);
	pros::delay(300);
	drive_velocity(0, 0);
	pros::delay(500);
	frontClip.set_value(true);
	pros::delay(200);


	PIDTurn(125);
	pros::delay(500);

	lift.move_velocity(100);
	pros::delay(2000);
	lift.move_velocity(0);
	pros::delay(100);

	drive_velocity(100, 100);
	pros::delay(1000);
	drive_velocity(0, 0);

	lift.move_velocity(-100);
	pros::delay(700);
	lift.move_velocity(0);

	frontClip.set_value(false);
	pros::delay(100);

	lift.move_velocity(100);
	pros::delay(300);
	lift.move_velocity(0);
	pros::delay(100);

	drive_velocity(-100, -100);
	pros::delay(300);
	drive_velocity(0, 0);
	lift.move_velocity(-100);
	pros::delay(1500);
	lift.move_velocity(0);

	PIDTurn(2);
	pros::delay(500);

	drive_velocity(50, 50);
	pros::delay(2000);
	drive_velocity(0, 0);
	frontClip.set_value(true);
	pros::delay(200);

	
	pather path3({
		{156, -47},
		{156, -70},

	}, 15, 75, true, 3000);
	pros::delay(100);

	PIDTurn(47);

	pather path3_t({
		{156, -70},
		{65, -153}
	}, 15, 75, true, 4000);
	
	

	/*
	pather path3_local({
		{0, 0},
		{-40, -47},
		{-44, -50},
		{-89, -95}
	}, 15, 75, true);
	*/

	
	pros::delay(100);

	pather path4({
		{61, -154},
		{84, -154},
		{84, -123}
	}, 10, false);

	pather path5({
		{84, -120},
		{84, -150}
	}, 10, 150, true, 2000);
	pros::delay(250);

	drive_velocity(-100, -100);
	pros::delay(500);
	drive_velocity(0,0);

	backClip.set_value(true);
	pros::delay(500);
	
	lift.move_velocity(100);

	pather path6({
		{82, -152},
		{82, -100}
	}, 15, 150, false, 3000);
	pros::delay(1000);

	//drive_velocity(-100, 100);
	//pros::delay(500);
	//drive_velocity(0, 0);
	//PIDTurn(-90);
	//pros::delay(300);
	lift.move_velocity(-100);
	//drive_velocity(50, 50);
	pros::delay(100);
	//drive_velocity(0, 0);
	pros::delay(500);
	lift.move_velocity(0);
	frontClip.set_value(false);
	pros::delay(100);
	lift.move_velocity(100);
	pros::delay(300);
	lift.move_velocity(0);
	
	//PIDTurn(0);

	pros::delay(100);
	//lift.move_velocity(-100);

	pather path7({
		{82, -44},
		{82, -80}
	}, 10, true);
	pros::delay(500);

	gyro.tare();
	turnPID(89, 0.25, 0.001, 1000);

	lift.move_velocity(-100);
	pros::delay(1500);
	

	pather path8({
		{123, -80},
		{144, -80}
	}, 10, false);
	pros::delay(500);
	frontClip.set_value(true);
	pros::delay(100);

	//turnPID(130, 0.25, 0.001, 1000);

	drive_voltage(6000, 6000);
	pros::delay(3000);
	drive_voltage(0, 0);

/*
	lift.move_velocity(100);
	pros::delay(2000);
	lift.move_velocity(0);

	pather path9({
		{146, -80},
		{171, -67},
		{187, -67}
	}, 15, false);

	frontClip.set_value(false);
	pros::delay(100);

	lift.move_velocity(100);
	pros::delay(750);
	lift.move_velocity(0);

	
	drive_velocity(-25, -25);
	pros::delay(250);
	drive_velocity(0, 0);
	*/
	
}

void tmp() {
	start_odom();

	
	pather test({
		{0, 0},
		{-13.04, 32.74},
		{-46.77, 47.48},
	}, 20, false);
}
