#include "functions.h"

void RightWinPoint()
{
  DriveTrainPID(700);

  TurnPID(-90);

  DriveTrainPID(-550);
  backClip.set_value(true);
  pros::delay(250);

  DriveTrainPID(50);

  RingIntake(4);
}

void LeftWinPoint()
{
  RingIntake(4);
}

void RightGrab()
{
  frontClip.set_value(false);

  DriveTrainPID(1400);
  frontClip.set_value(true);

  DriveTrainPID(200);

  TurnPID(90);
}

void LeftGrab()
{
  frontClip.set_value(false);

  DriveTrainPID(1400);
  frontClip.set_value(true);

  DriveTrainPID(200);

  TurnPID(-90);
}

void RightFull()
{
  frontClip.set_value(false);

  DriveTrainPID(1400);
  frontClip.set_value(true);

  DriveTrainPID(625);

  TurnPID(-90);

  DriveTrainPID(-400);
  backClip.set_value(true);
  pros::delay(250);

  DriveTrainPID(50);

  RingIntake(4);
}

void LeftFull()
{
  DriveTrainPID(24.4);

  TurnPID(-90);

  DriveTrainPID(-19.2);
  backClip.set_value(true);
  pros::delay(250);

  DriveTrainPID(1.75);

  RingIntake(20);
}

void DoubleGrab()
{
  frontClip.set_value(false);

  lift.move_velocity(-50);

  DriveTrainPID(50.25);
  frontClip.set_value(true);
  pros::delay(100);
  lift.move_velocity(0);

  DriveTrainPID(15);

  TurnPID(-180);
  frontClip.set_value(false);
  DriveTrainPID(-17);

  TurnPID(-56);

  DriveTrainPID(28.5);
  frontClip.set_value(true);
  pros::delay(100);

  DriveTrainPID(-24);

  WallPush();
  backClip.set_value(true);

  intake.move_velocity(400);

  TurnPID(-20);
  DriveTrainPID(-10);

  intake.move_velocity(0);
}

void FullGrab()
{

}

void Skills()
{
  frontClip.set_value(false);

  DriveTrainPID(1400);
  frontClip.set_value(true);

  DriveTrainPID(570);

  TurnPID(-90);

	DriveTrainPID(200);

	Lift(up);

	TurnPID(-180);

	DriveTrainPID(250);

	frontClip.set_value(false);

	//DriveTrainPID(-25);

	TurnPID(87);

	Lift(down);

	DriveTrainPID(1200);

	frontClip.set_value(true);

	DriveTrainPID(750);

	TurnPID(-8);
}

void None()
{

}
