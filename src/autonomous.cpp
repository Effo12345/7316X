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
	frontClip.set_value(false);

	pather fastGrab({
		{-57.5, -36.25},
		{-17.5, -36.25}
	}, 30, 1000, 2.5, false, false, 2000);	
	frontClip.set_value(true);

	pros::delay(100);

	while(Units.DegToIn(tracking.get_value()) > 10)
		drive_voltage(-12000, -12000);

	drive_voltage(0, 0);
}

//Primary autons
void LeftFull() {
	stateMachine.setHoldGoal();
  	stateMachine.setGuard();
	
	set_odom(-55.75, 44.5, 82.69);
	start_odom();

	clipGuard.set_value(true);
	frontClip.set_value(false);
	pather fastGrab({
		{-55.75, 44.5},
        {-17.44, 37.06}
	}, 30, 800, 2.5, false, false, 2000);
	
	//pros::delay(50);
	frontClip.set_value(true);
	pros::delay(100);

	pather path1({
		{0, 36.14},
        {-40.25, 47.76},
        {-59.24, 31.89}
	}, 20, true);
	backClip.set_value(true);
	pros::delay(300);

	lift.move_velocity(100);
	pather path2({
		{-53.86, 33.87},
		{-40.35, 47.76}
	}, 10, false);

	
	PIDTurn(-165);
	pros::delay(1000);
	if(gyro.get_pitch() > 50) {
		lift.move_velocity(-100);
		pros::delay(5000);
	}
	else {
		ringIntake(on);
		ringMove(2500);
		lift.move_velocity(0);

		drive_velocity(-100, -100);
		pros::delay(500);
		drive_velocity(0, 0);
		backClip.set_value(false);
		ringIntake(off);
	}
}

void RightFull() {
  	stateMachine.setHoldGoal();
  	stateMachine.setGuard();

  	set_odom(-57.5, -36.25, 90);
	start_odom();
	
	clipGuard.set_value(true);
	frontClip.set_value(false);

	pather fastGrab({
		{-57.5, -36.25},
		{-17.5, -36.25}
	}, 30, 1000, 2.5, false, false, 2000);	
	frontClip.set_value(true);

	pros::delay(100);
	
	pather path({
		{0, -35.29},
		{-33.17, -26.79},
		{-35.15, -57.97}
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
		{2.0, -50.05}
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
	frontClip.set_value(false);
	hyperGrab(48);
	pros::delay(200);
	frontClip.set_value(true);
	pros::delay(500);

	pather path1({
		{0, 0},
		{-49.32, -68.46}
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
	frontClip.set_value(false);
	pros::delay(200);
	backClip.set_value(false);
	pros::delay(500);

	pather path1({
		{-59.24, 37.84},
		{-40.82, 41.53},
		{-44.5, -43.23}
	}, 15, false);
	pros::delay(200);

	PIDTurn(20);
	pros::delay(100);

	pather path2({
		{-43.65, -41.53},
		{-44.5, -70.16}
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
		{-10.77, -16.3}
	}, 20, false);
	pros::delay(100);

	frontClip.set_value(true);
	pros::delay(100);

	pather path4({
		{-9.92, -14.88},
		{-52.16, -46.91}
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
	frontClip.set_value(false);
	pather fastGrab({
		{-57.5, -36.25},
		{-17.5, -36.25}
	}, 30, 1000, 2.5, false, false, 2000);
	frontClip.set_value(true);
	pros::delay(50);
	lift.move_velocity(10);

	pather path1({
		{0, -35.01},
		{-31.18, -36.43},
		{-17.57, -14.6},
		{-6.5, -8.93}
	}, 20, 200, 8, true, 3000);
	lift.move_velocity(0);

	backClip.set_value(true);
	pros::delay(300);

	pather path2({
		{-0.28, 0.14},
		{-32.88, -11.2}
	}, 20, false);

	PIDTurn(90);
	backClip.set_value(false);
	pros::delay(100);

	pather path3({
		{-28.91, -12.9},
		{-28.91, -38.41}
	}, 20, false);

	PIDTurn(-150);
	pather path4({
		{-36, -33.87},
		{-32, -68.74}
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
	set_odom(-57.5, -36.25, 90);
	start_odom();

	frontClip.set_value(false);

	pather fastGrab({
		{-57.5, -36.25},
		{-17.5, -36.25}
	}, 30, 1000, 2.5, false, false, 2000);
	frontClip.set_value(true);
	pros::delay(50);
	lift.move_velocity(10);

	pather tallGrab({
		{0, -35.01},
		{-31.18, -36.43},
		{-17.57, -14.6},
		{-6.5, -8.93}
	}, 20, 200, 8, true, 3000);
	lift.move_velocity(0);

	backClip.set_value(true);
	pros::delay(300);

	pather tallToHomeZone({
		{-0.28, 0.14},
		{-32.88, -11.2}
	}, 20, false);

	PIDTurn(90);
	backClip.set_value(false);
	pros::delay(100);

	pather towardsAlliance({
		{-28.91, -12.9},
		{-28.91, -38.41}
	}, 20, false);

	PIDTurn(-150);
	pather allianceGrab({
		{-36, -33.87},
		{-32, -68.74}
	}, 20, true);
	backClip.set_value(true);
	pros::delay(500);

	lift.move_velocity(100);
	PIDTurn(-150);
	ringIntake(on);
	pros::delay(500);
	pros::delay(500);
	ringMove(2500);
	lift.move_velocity(0);

	drive_voltage(-6000, -6000);
	pros::delay(400);
	drive_voltage(0, 0);
	PIDTurn(0);
	pros::delay(100);
	PIDTurn(10);
	pros::delay(100);
	
	pather ringLine({
		{-70.3, -47.2},
        {-24.09, -50.6},
        {1.13, -52.02}
	}, 20, 30, false, 3000);
	pros::delay(100);

	pather towardsPlatform({
		{0.57, -50.6},
        {30.61, -46.63},
        {28.06, -0.43}
	}, 20, false);
	ringIntake(off);
	pros::delay(100);

	PIDTurn(0);
	pros::delay(100);

	drive_voltage(6000, 6000);
	pros::delay(250);
	drive_voltage(0, 0);

	lift.move_velocity(-100);
	pros::delay(2000);
	frontClip.set_value(false);
	lift.move_velocity(100);
	pros::delay(700);
	lift.move_velocity(0);

	PIDTurn(-90);
	pros::delay(100);
	lift.move_velocity(-100);
	backClip.set_value(false);
	pros::delay(300);

	drive_voltage(6000, 6000);
	pros::delay(250);
	drive_voltage(0, 0);

	PIDTurn(90);
	pros::delay(100);
	PIDTurn(90);
	pros::delay(100);

	pather allianceGrab2({
		{37.98, -2.41},
        {36.57, 32.74},
        {37.98, 70.44}
	}, 20, true);
	pros::delay(100);

	backClip.set_value(true);
	pros::delay(300);

	pather neutralGrab({
		{35.15, 57.97},
        {42.8, 30.76},
        {-2.83, 33.59}
	}, 20, false);
	pros::delay(200);
	frontClip.set_value(true);
	pros::delay(100);

	lift.move_velocity(10);
	pather moveBack({
		{0, 36.14},
        {35.43, 33.87},
        {50.74, 48.05}
	}, 20, true);
	lift.move_velocity(100);
	pros::delay(100);

	PIDTurn(180);
	pros::delay(100);

	ringIntake(on);
	pather ringGrab({
		{50.74, 48.05},
        {24.38, 48.05},
        {-2, 50.05},
        {-68.88, 47.76}
	}, 20, 30, false, 10000);
	pros::delay(100);
	ringIntake(off);

	pather ringSafety({
		{-70.3, 48.33},
        {-40.54, 55.42},
        {-34.58, 31.89}
	}, 20, true);
	pros::delay(100);

	PIDTurn(90);
	pros::delay(100);
	PIDTurn(90);
	pros::delay(100);

	pather stack2({
		{-34.58, 31.89},
        {-30.9, 18.28},
        {-43.65, 8.36}
	}, 20, false);

	PIDTurn(130);
	pros::delay(100);
	PIDTurn(130);
	pros::delay(100);

	drive_voltage(6000, 6000);
	pros::delay(500);
	drive_voltage(0, 0);

	PIDTurn(180);
	pros::delay(100);

	drive_voltage(6000, 6000);
	pros::delay(250);
	drive_voltage(0, 0);

	lift.move_velocity(-100);
	pros::delay(2000);
	frontClip.set_value(false);
	lift.move_velocity(100);

	PIDTurn(90);
	pros::delay(100);
	PIDTurn(90);
	pros::delay(100);

	lift.move_velocity(-100);
	drive_velocity(-6000, -6000);
	pros::delay(500);
	drive_velocity(0, 0);
	pros::delay(500);

	backClip.set_value(false);
	pros::delay(300);

	drive_voltage(6000, 6000);
	pros::delay(100);
	drive_voltage(0, 0);

	PIDTurn(140);
	pros::delay(100);
	PIDTurn(140);
	pros::delay(100);

	drive_voltage(6000, 6000);
	pros::delay(600);
	drive_voltage(0, 0);
	pros::delay(100);

	frontClip.set_value(true);
	pros::delay(100);

	pather towardsRed({
		{-58.39, 31.61},
        {51.31, 39.83}
	}, 20, true);
}


