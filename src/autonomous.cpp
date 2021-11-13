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
  frontClip.set_value(false);

  DriveTrainPID(1475);
	frontClip.set_value(true);

	DriveTrainPID(550);

	TurnPID(-45);

	DriveTrainPID(-450);
	pros::delay(250);

	RingIntake(5);
}

void DoubleGrab()
{
  frontClip.set_value(false);

  DriveTrainPID(1400);
  frontClip.set_value(true);


  DriveTrainPID(625);

  TurnPID(180);
  frontClip.set_value(false);

  DriveTrainPID(-200);

  TurnPID(-49);

  DriveTrainPID(850);
  frontClip.set_value(true);

  DriveTrainPID(-400);
}

void Skills()
{
  frontClip.set_value(false);

  DriveTrainPID(1400);
  frontClip.set_value(true);

  DriveTrainPID(570);

  TurnPID(-90);

	DriveTrainPID(00);

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
