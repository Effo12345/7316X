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
	hyperGrab(42);
	frontClip.set_value(true);
	while(Units.DegToIn(tracking.get_value()) > 10)
		drive_voltage(-12000, -12000);

	drive_voltage(0, 0);
}

void RightGrab() {
  	stateMachine.setHoldGoal();
  	stateMachine.setGuard();

  	clipGuard.set_value(true);
	hyperGrab(39.5);
	frontClip.set_value(true);
}

//Primary autons
void LeftFull() {
	stateMachine.setHoldGoal();
  	stateMachine.setGuard();
	
	set_odom(-55.75, 44.5, 82.69);
	start_odom();

	clipGuard.set_value(true);
	frontClip.set_value(false);
	hyperGrab(42);
	frontClip.set_value(true);
	pros::delay(50);

	pather path1({
		{0, 36.14},
		{-35.15, 43.23},
		{-59.24, 29.06},
	}, 20, true);
	backClip.set_value(true);
	pros::delay(300);

	lift.move_velocity(100);
	pather path2({
		{-53.86, 33.87},
		{-40.35, 47.76},
	}, 10, false);

	
	PIDTurn(-165);
	pros::delay(1000);
	ringIntake(on);
	ringMove(2500);
	lift.move_velocity(0);

	drive_velocity(-100, -100);
	pros::delay(500);
	drive_velocity(0, 0);
	backClip.set_value(false);
	ringIntake(off);
}

void RightFull() {
  	stateMachine.setHoldGoal();
  	stateMachine.setGuard();

  	set_odom(-57.5, -36.25, 90);
	start_odom();
	
	clipGuard.set_value(true);
	hyperGrab(39.5);
	frontClip.set_value(true);
	//initTOW();
	pros::delay(100);
	
	pather path({
		{0, -35.29},
		{-33.17, -26.79},
		{-35.15, -57.97},
	}, 20, true);

	pros::delay(100);
	backClip.set_value(true);
	//stopTOW();

	lift.move_velocity(100);
	PIDTurn(-10);
	lift.move_velocity(0);
	ringIntake(on);

	pather path2({
		{-35.43, -59.39},
		{2.0, -50.05},
	}, 20, 30, false, 3000);

	drive_velocity(-100, -100);
	pros::delay(1250);
	drive_velocity(0, 0);
	backClip.set_value(false);

	//tOW.join();
	//drive_voltage(-12000, -12000);
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
  	translatePID(-24);
	pros::delay(100);
	backClip.set_value(true);
	pros::delay(500);

	lift.move_velocity(100);
	pros::delay(200);
	lift.move_velocity(0);
	ringIntake(on);

	translatePID(-5);
	pros::delay(500);
	ringIntake(off);
	backClip.set_value(false);
}

void RightTall() {
	stateMachine.setHoldGoal();
  	stateMachine.setGuard();

	set_odom(-60.25, -30.5, 90);
	start_odom();

	clipGuard.set_value(true);
	hyperGrab(48);
	pros::delay(200);
	frontClip.set_value(true);
	pros::delay(500);

	pather path1({
		{0, 0},
		{-49.32, -68.46},
	}, 20, 100, 7, true, 3000);

	lift.move_velocity(100);
	pros::delay(100);
	backClip.set_value(true);

	PIDTurn(20);
	lift.move_velocity(0);
	ringIntake(on);
	lift.move_velocity(0);
	pros::delay(500);
	
	pather path2({
		{-46.85, -63.84},
		{-5, -85.84}
	}, 20, 30, false, 3000);

	drive_velocity(-100, -100);
	pros::delay(1500);
	drive_velocity(0, 0);
	ringIntake(off);
	backClip.set_value(false);
}

void FullWinPoint() {
	set_odom(-58, 37.5, 0);
	start_odom();

	backClip.set_value(true);
	pros::delay(200);
	backClip.set_value(false);
	pros::delay(500);

	pather path1({
		{-59.24, 37.84},
		{-40.82, 41.53},
		{-44.5, -43.23},
	}, 15, false);
	pros::delay(200);

	PIDTurn(20);
	pros::delay(100);

	pather path2({
		{-43.65, -41.53},
		{-44.5, -70.16},
	}, 20, true);
	pros::delay(100);
	
	backClip.set_value(true);
	pros::delay(300);
	lift.move_velocity(100);
	ringIntake(on);
	pros::delay(100);
	lift.move_velocity(0);
	pros::delay(500);
	ringIntake(off);
	lift.move_velocity(-100);

	pather path3({
		{-43.65, -57.69},
		{-25.23, -22.25},
		{-10.77, -16.3},
	}, 20, false);
	pros::delay(100);

	frontClip.set_value(true);
	pros::delay(100);

	pather path4({
		{-9.92, -14.88},
		{-52.16, -46.91},
	}, 20, true);
	pros::delay(100);
	backClip.set_value(false);
}


//Double neutral grab
void DoubleGrab() {
	stateMachine.setHoldGoal();
  	stateMachine.setGuard();

  	set_odom(-57.5, -36.25, 90);
	start_odom();


  	clipGuard.set_value(true);
	hyperGrab(39.5);
	frontClip.set_value(true);
	pros::delay(50);
	lift.move_velocity(10);

	pather path1({
		{0, -35.01},
		{-31.18, -36.43},
		{-17.57, -14.6},
		{-6.5, -8.93},
	}, 20, 200, 8, true, 3000);
	lift.move_velocity(0);

	backClip.set_value(true);
	pros::delay(300);

	pather path2({
		{-0.28, 0.14},
		{-32.88, -11.2},
	}, 20, false);

	PIDTurn(90);
	backClip.set_value(false);
	pros::delay(100);

	pather path3({
		{-28.91, -12.9},
		{-28.91, -38.41},
	}, 20, false);

	PIDTurn(-150);
	pather path4({
		{-36, -33.87},
		{-32, -68.74},
	}, 20, true);
	backClip.set_value(true);
	pros::delay(500);

	lift.move_velocity(100);
	PIDTurn(-150);
	ringIntake(on);
	pros::delay(500);
	pros::delay(500);
	ringMove(2000);
	lift.move_velocity(0);

	drive_voltage(-200, -200);
	pros::delay(600);
	drive_voltage(0, 0);
	PIDTurn(200);
	backClip.set_value(false);
	ringIntake(off);
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
	//calibrate_odom();
	
	/*
	initTOW();

	tOW.join();
	drive_voltage(-12000, -12000);
	*/

	
	set_odom(-57.5, -36.25, 90);
	start_odom();
	
	clipGuard.set_value(true);

	pather fastGrab({
		{-57.5, -36.25},
		{-17.5, -36.25}
	}, 30, 1000, 2.5, false, false, 2000);	
	frontClip.set_value(true);

	pros::delay(100);
	
	pather path({
		{0, -35.29},
		{-33.17, -26.79},
		{-35.15, -57.97},
	}, 20, true);

	pros::delay(100);
	backClip.set_value(true);
	//stopTOW();

	lift.move_velocity(100);
	PIDTurn(-10);
	lift.move_velocity(0);
	ringIntake(on);

	pather path2({
		{-35.43, -59.39},
		{2.0, -50.05},
	}, 20, 30, false, 3000);

	drive_velocity(-100, -100);
	pros::delay(1250);
	drive_velocity(0, 0);
	backClip.set_value(false);


	/*
	clipGuard.set_value(true);
	ringIntake(on);
	lift.move_velocity(25);
	drive_voltage(12000, 12000);
	while(tracking.get_value() < 2007)
		pros::delay(10);
	ringIntake(off);
	lift.move_velocity(-100);
	hyperGrab(50);
	pros::delay(300);
	frontClip.set_value(true);
	pros::delay(100);
	*/
	//turnTo(10, 10, false);
}
