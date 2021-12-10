#include "functions.h"

void RightWinPoint()
{
  DriveTrainPID(35);

	FineTurn(-45);

	WallPush(1000);
	RingIntake(7);
}

void LeftWinPoint()
{
  RingIntake(7);
}

void RightGrab()
{
  frontClip.set_value(false);

  DriveTrainPID(1400);
  frontClip.set_value(true);

  DriveTrainPID(200);

  CoarseTurn(90);
}

void LeftGrab()
{
  frontClip.set_value(false);
	DriveTrainPID(49);
	pros::delay(100);
	frontClip.set_value(true);
	DriveTrainPID(8);

	FineTurn(-52);
	WallPush(250);

	RingIntake(7);
	pros::delay(500);
	DriveTrainPID(10);
}

void RightFull()
{
  frontClip.set_value(false);
	lift.move_velocity(-50);

	ArcMove(1375, 0.28, left);

	FineTurn(-58);

	DriveTrainPID(24.5); //19 original

	frontClip.set_value(true);
	pros::delay(100);
	lift.move_velocity(0);

	DriveTrainPID(-20);

	WallPush(1400);

	RingIntake(7);
	pros::delay(100);

	DriveTrainPID(15);
}

void LeftFull()
{
  frontClip.set_value(false);

	RingIntake(3);

	DriveTrainPID(10);
	CoarseTurn(135);

	DriveTrainPID(64.5);
	frontClip.set_value(true);
	pros::delay(100);

	DriveTrainPID(10);
}

void DoubleGrab()
{
  frontClip.set_value(false);

  lift.move_velocity(-50);

  DriveTrainPID(49.5);
  frontClip.set_value(true);
  pros::delay(100);
  lift.move_velocity(0);

  DriveTrainPID(15);

  CoarseTurn(-180);
  frontClip.set_value(false);
  DriveTrainPID(-17);

  FineTurn(-60);

  DriveTrainPID(30.5);
  frontClip.set_value(true);
  pros::delay(100);

  DriveTrainPID(-21.0);

  WallPush(1000);
  backClip.set_value(true);

  intake.move_velocity(400);

  FineTurn(-20);
  DriveTrainPID(-10);

  intake.move_velocity(0);
}

void FullWinPoint()
{
  backClip.set_value(true);
	pros::delay(700);

	DriveTrainPID(15);
	pros::delay(100);

	FineTurn(-45, 150, 0.0);
	pros::delay(100);
	ArcMove(-1000, .492, left);

	backClip.set_value(false);
	pros::delay(100);


	WallPush(1700);
	backClip.set_value(true);
	RingIntake(7);

	DriveTrainPID(20);
	backClip.set_value(false);
	DriveTrainPID(45);
}

void Skills()
{
  frontClip.set_value(false);
  backClip.set_value(false);

  DriveTrainPID(23);
  PTurn(-90);

  WallPush(400);
  pros::delay(25);
  backClip.set_value(true);

  DriveTrainPID(6);
  PTurn(0);

  DriveTrainPID(67);
  frontClip.set_value(true);
  pros::delay(100);

  DriveTrainPID(64);
  PTurn(-90);
  backClip.set_value(false);

  DriveTrainPID(12);
  PTurn(-180);
}

void None()
{

}
